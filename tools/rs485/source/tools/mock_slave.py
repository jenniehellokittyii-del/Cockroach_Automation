# -*- coding: utf-8 -*-
"""
模拟 Modbus 从站（TCP）—— 无硬件时的"假变送器"，给上位机提供测试数据。

用途：
  1. 上位机 GUI 选「TCP 模拟」模式连它，整条采集链路不用硬件就能跑通；
  2. tools/loopback_test.py 里 import 本模块自动起停，做自动化自检；
  3. 也可以单独运行，当一台常驻的模拟设备：
       .venv\\Scripts\\python.exe tools\\mock_slave.py --signal puncture

模拟信号（--signal 参数）：
  puncture  穿刺曲线：静息 -> 匀速加载 -> 刺穿瞬间跌落 -> 回零，8 秒一循环（默认）
  sine      正弦波 0~5 N
  static    恒定 2.00 N

寄存器表（与真变送器约定一致）：
  hr[0]      力值原始值 = 力(N) × 100，16 位，比如 752 表示 7.52 N
  hr[1]      固定 0（留位）
  hr[2..3]   32 位版本：力(N) × 10000，高字在前（测试 32 位读数用）
  hr[0x9C40..0x9C41]  32 位力值窗口：力(N) × 10000，高字在前
              （默认地址，与真变送器寄存器 40000 一致；--base-register 可改）
  hr[17]     固定 12345（验证寄存器地址对不对）
  hr[19]     固定 6789
  hr[4..16, 18, 20..63]  可读可写（写进去能读出来，测试 06 功能码用）
  ir[0]      和 hr[0] 同步（测试 04 功能码用）

只响应从站地址 --unit（默认 1），别的地址不理（和真设备一致，方便测超时）。
"""

from __future__ import annotations

import argparse
import math
import random
import socket
import struct
import threading
import time

from modbus_link import hex_frame  # 同目录 tools/ 下运行或上层 import 都能用


# ==================== 仿真信号 ====================

class Signal:
    """生成力值曲线（单位 N）。"""

    def __init__(self, mode: str = 'puncture', noise: float = 0.02):
        self.mode = mode
        self.noise = noise
        self._t0 = time.monotonic()

    def force(self) -> float:
        t = time.monotonic() - self._t0
        if self.mode == 'static':
            base = 2.0
        elif self.mode == 'sine':
            base = 2.5 + 2.5 * math.sin(2 * math.pi * t / 4.0)
        else:
            base = self._puncture(t)
        return max(0.0, base + random.gauss(0, self.noise))

    @staticmethod
    def _puncture(t: float) -> float:
        """8 秒一个穿刺循环：2s 静息 -> 3s 加载到 7.5N -> 0.15s 跌落 -> 回零。"""
        p = t % 8.0
        if p < 2.0:
            return 0.0
        if p < 5.0:
            return 7.5 * (p - 2.0) / 3.0
        if p < 5.15:
            return 1.5
        if p < 6.5:
            return 1.5 - 1.2 * (p - 5.15) / 1.35
        return 0.3 - 0.3 * (p - 6.5) / 1.5


# ==================== 从站实现 ====================

def _recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionResetError('对端关闭')
        buf += chunk
    return bytes(buf)


class MockSlave:
    """极简 Modbus TCP 从站，够用就好：03/04/06/10 + 异常帧。"""

    def __init__(self, host: str = '127.0.0.1', port: int = 5020, unit: int = 1,
                 signal: str = 'puncture', verbose: bool = True,
                 base_register: int = 0x9C40):
        self.host = host
        self.port = port
        self.unit = unit
        self.signal = Signal(signal)
        self.verbose = verbose
        self.base_register = int(base_register)

        self.regs = [0] * 64
        self.regs[17] = 12345
        self.regs[19] = 6789

        self._srv: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ---------- 生命周期 ----------

    def start(self) -> 'MockSlave':
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        if self.port == 0:  # 传 0 表示让系统分配空闲端口（避免和已开的实例撞车）
            self.port = srv.getsockname()[1]
        srv.listen(4)
        srv.settimeout(0.2)
        self._srv = srv
        self._thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._thread.start()
        if self.verbose:
            print(f'[模拟从站] 已启动 {self.host}:{self.port}  从站地址={self.unit}  '
                  f'信号={self.signal.mode}  32位力值窗口=0x{self.base_register:X}')
        return self

    def stop(self) -> None:
        self._stop.set()
        if self._srv is not None:
            try:
                self._srv.close()
            except OSError:
                pass
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        if self.verbose:
            print('[模拟从站] 已停止')

    # ---------- 内部 ----------

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                conn, _addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn: socket.socket) -> None:
        conn.settimeout(0.5)
        with conn:
            while not self._stop.is_set():
                try:
                    head = _recv_exact(conn, 7)
                except socket.timeout:
                    continue
                except (OSError, ConnectionResetError):
                    return
                txn, proto, length, unit = struct.unpack('>HHHB', head)
                if proto != 0 or length < 2:
                    return
                try:
                    pdu = _recv_exact(conn, length - 1)
                except (OSError, ConnectionResetError):
                    return
                if self.verbose:
                    print(f'  RX {hex_frame(head + pdu)}')
                if unit != self.unit:
                    continue  # 不是找我的，不理（真设备也这样）
                resp_pdu = self._handle(pdu)
                if resp_pdu is None:
                    continue
                resp = struct.pack('>HHHB', txn, 0, len(resp_pdu) + 1, unit) + resp_pdu
                if self.verbose:
                    print(f'  TX {hex_frame(resp)}')
                try:
                    conn.sendall(resp)
                except OSError:
                    return

    def _live_value(self, addr: int) -> int:
        raw16 = int(round(self.signal.force() * 100)) & 0xFFFF
        raw32 = int(round(self.signal.force() * 10000)) & 0xFFFFFFFF
        if addr == self.base_register:
            return (raw32 >> 16) & 0xFFFF
        if addr == self.base_register + 1:
            return raw32 & 0xFFFF
        if addr == 0:
            return raw16
        if addr == 2:
            return (raw32 >> 16) & 0xFFFF
        if addr == 3:
            return raw32 & 0xFFFF
        return self.regs[addr]

    def _readable(self, addr: int, count: int) -> bool:
        """可读范围：常规寄存器 0..63 + 32 位力值窗口（默认 0x9C40/0x9C41）。"""
        for a in range(addr, addr + count):
            if 0 <= a < len(self.regs):
                continue
            if a in (self.base_register, self.base_register + 1):
                continue
            return False
        return True

    def _handle(self, pdu: bytes) -> bytes | None:
        func = pdu[0]

        # 03 读保持寄存器 / 04 读输入寄存器
        if func in (3, 4):
            addr, count = struct.unpack('>HH', pdu[1:5])
            if count < 1 or count > 125 or not self._readable(addr, count):
                return bytes([func | 0x80, 0x02])  # 非法数据地址
            vals = [self._live_value(a) for a in range(addr, addr + count)]
            data = b''.join(struct.pack('>H', v) for v in vals)
            return bytes([func, len(data)]) + data

        # 06 写单个寄存器
        if func == 6:
            addr, val = struct.unpack('>HH', pdu[1:5])
            if addr >= len(self.regs) or addr in (0, 2, 3):
                return bytes([func | 0x80, 0x02])
            self.regs[addr] = val
            return pdu[:5]

        # 10 写多个寄存器
        if func == 0x10:
            addr, count, nbytes = struct.unpack('>HHB', pdu[1:6])
            if count < 1 or count > 123 or nbytes != count * 2 or addr + count > len(self.regs):
                return bytes([func | 0x80, 0x03])
            for i in range(count):
                self.regs[addr + i] = struct.unpack('>H', pdu[6 + i * 2:8 + i * 2])[0]
            return struct.pack('>BHH', func, addr, count)

        return bytes([func | 0x80, 0x01])  # 非法功能码


def main() -> None:
    ap = argparse.ArgumentParser(description='模拟 Modbus TCP 从站（无硬件测试用）')
    ap.add_argument('--host', default='127.0.0.1')
    ap.add_argument('--port', type=int, default=5020)
    ap.add_argument('--unit', type=int, default=1, help='从站地址（默认 1）')
    ap.add_argument('--signal', choices=['puncture', 'sine', 'static'], default='puncture')
    ap.add_argument('--quiet', action='store_true', help='不打印每一帧')
    ap.add_argument('--base-register', type=lambda x: int(x, 0), default=0x9C40,
                    help='32 位力值窗口起始寄存器（默认 0x9C40，与真变送器一致）')
    args = ap.parse_args()

    slave = MockSlave(args.host, args.port, args.unit, args.signal,
                      verbose=not args.quiet, base_register=args.base_register).start()
    print('按 Ctrl+C 停止')
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        slave.stop()


if __name__ == '__main__':
    main()
