# -*- coding: utf-8 -*-
"""RS485 力/称重传感器读取模块（钻孔阶跃检测的数据源）。

职责：
  - 后台线程轮询 Modbus 保持寄存器（默认 0x9C40，共 2 个），
    组装 32 位有符号值（高字在前），写入环形缓冲与 CSV；
  - latest() 给钻孔状态机取最新 (monotonic, raw, value)；
  - 连续失败计数 + 数据过期判定，供状态机识别 SENSOR_FAULT；
  - 支持 serial / tcp 两种链路（tcp 供无硬件本地测试）；
  - Modbus 组帧/解析复用 tools/rs485/source/modbus_link.py（懒加载）。

现场注意：
  - 传感器串口是独占资源：运行本模块时不要再开 RS485 GUI 或串口助手；
  - COM 口以设备管理器实际为准，在 config/force_sensor.json 覆盖默认值；
  - 读数比例（原始值 -> 物理量）用砝码标定后填 scale。

无硬件测试：
  - 单测见 tests/test_force_sensor.py（注入假链路）；
  - 端到端见 tests/test_e2e_mock_slave.py（TCP 连 tools/rs485/source/tools/mock_slave.py）。
"""

from __future__ import annotations

import csv
import json
import sys
import threading
import time
from collections import deque
from pathlib import Path

from settings import PROJECT_ROOT, FORCE_SENSOR_CONFIG

CONFIG_PATH = FORCE_SENSOR_CONFIG

DEFAULT_CONFIG = {
    'mode': 'serial',          # serial | tcp
    'port': 'COM21',           # 传感器 COM 口（现场核对后按需覆盖）
    'baudrate': 19200,         # 已现场确认
    'bytesize': 8,
    'parity': 'N',
    'stopbits': 1,
    'host': '127.0.0.1',       # tcp 模式（本地测试）
    'tcp_port': 5020,
    'unit': 1,                 # 从站地址
    'register': 0x9C40,        # 40000，已实测
    'count': 2,                # 32 位 = 2 个寄存器
    'high_word_first': True,
    'signed': True,
    'scale': 1.0,              # 换算值 = 原始值 × scale（砝码标定后填）
    'timeout': 0.3,
    'poll_interval': 0.0,      # 每帧额外等待；0 = 尽快轮询（链路阻塞自然限速）
    'buffer_samples': 5000,
}


def load_config(path=None):
    """默认配置 + 可选 JSON 覆盖（现场只改 config/force_sensor.json，不动代码）。"""
    config = dict(DEFAULT_CONFIG)
    candidate = Path(path) if path else CONFIG_PATH
    if candidate.exists():
        overlay = json.loads(candidate.read_text(encoding='utf-8'))
        config.update(overlay)
    return config


def find_modbus_link_dir():
    """定位 modbus_link.py：优先整理版仓库布局，兼容原工作区布局。"""
    for candidate in (
        PROJECT_ROOT / 'tools' / 'rs485' / 'source',
        PROJECT_ROOT / 'RS485-USB-SERIAL-READ',
    ):
        if (candidate / 'modbus_link.py').exists():
            return candidate
    return None


def combine_words(words, high_word_first=True, signed=True):
    """把两个 16 位寄存器组装成 32 位值（默认高字在前、有符号）。"""
    if len(words) != 2:
        raise ValueError(f'需要 2 个寄存器，实际收到 {len(words)} 个')
    hi, lo = (words[0], words[1]) if high_word_first else (words[1], words[0])
    raw = ((int(hi) & 0xFFFF) << 16) | (int(lo) & 0xFFFF)
    if signed and raw >= 0x80000000:
        raw -= 0x100000000
    return raw


class ForceSensor:
    """一条传感器链路：后台轮询 + 环形缓冲 + CSV 落盘。

    测试可注入 link（形如 modbus_link.ModbusLink 的对象，需提供
    read_registers(addr, count)；注入时不负责 open/close 链路）。
    """

    def __init__(self, config=None, csv_path=None, link=None):
        self.config = load_config() if config is None else {**DEFAULT_CONFIG, **config}
        stamp = time.strftime('%Y%m%d_%H%M%S')
        self.csv_path = (
            Path(csv_path) if csv_path
            else PROJECT_ROOT / 'runtime' / 'motion_logs' / f'sensor_{stamp}.csv'
        )
        self._link = link
        self._link_owned = link is None
        self._samples = deque(maxlen=int(self.config['buffer_samples']))
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._csv_file = None
        self._csv_writer = None
        self.consecutive_failures = 0
        self.total_failures = 0
        self.total_samples = 0
        self.last_error = None

    # ---------- 链路与文件 ----------

    def open(self):
        if self._link is None:
            link_dir = find_modbus_link_dir()
            if link_dir is None:
                raise RuntimeError(
                    '找不到 modbus_link.py（检查 tools/rs485/source 目录是否存在）'
                )
            if str(link_dir) not in sys.path:
                sys.path.insert(0, str(link_dir))
            import modbus_link as mb
            cfg = self.config
            self._link = mb.ModbusLink(
                mode=cfg['mode'],
                port=cfg['port'],
                baudrate=int(cfg['baudrate']),
                bytesize=int(cfg['bytesize']),
                parity=cfg['parity'],
                stopbits=int(cfg['stopbits']),
                host=cfg['host'],
                tcp_port=int(cfg['tcp_port']),
                unit=int(cfg['unit']),
                timeout=float(cfg['timeout']),
            )
            self._link_owned = True
        self._link.open()
        self.csv_path.parent.mkdir(parents=True, exist_ok=True)
        self._csv_file = self.csv_path.open('w', newline='', encoding='utf-8')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(['iso_time', 'monotonic', 'raw', 'value'])
        self._csv_file.flush()

    def close(self):
        self.stop()
        if self._csv_file is not None:
            try:
                self._csv_file.close()
            finally:
                self._csv_file = None
                self._csv_writer = None
        if self._link is not None and self._link_owned:
            try:
                self._link.close()
            finally:
                self._link = None

    # ---------- 采集线程 ----------

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        try:
            self.open()
        except Exception:
            self._stop_event.set()
            if self._link is not None and self._link_owned:
                try:
                    self._link.close()
                except Exception:
                    pass
                self._link = None
            raise
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def read_once(self):
        """同步读一帧，返回 (raw, value)；失败直接抛异常。"""
        if self._link is None:
            raise RuntimeError('链路未打开')
        cfg = self.config
        words = self._link.read_registers(int(cfg['register']), int(cfg['count']))
        raw = combine_words(words, cfg['high_word_first'], cfg['signed'])
        value = raw * float(cfg['scale'])
        return raw, value

    def _poll_loop(self):
        interval = float(self.config['poll_interval'])
        while not self._stop_event.is_set():
            frame_started = time.monotonic()
            try:
                raw, value = self.read_once()
                now = time.monotonic()
                with self._lock:
                    self._samples.append((now, raw, value))
                self.consecutive_failures = 0
                self.last_error = None
                self.total_samples += 1
                if self._csv_writer is not None:
                    self._csv_writer.writerow(
                        [time.strftime('%Y-%m-%dT%H:%M:%S'), f'{now:.6f}', raw, value]
                    )
                    self._csv_file.flush()
            except Exception as exc:  # 链路错误：计数并继续，由上层判 SENSOR_FAULT
                self.consecutive_failures += 1
                self.total_failures += 1
                self.last_error = str(exc)
            wait = interval - (time.monotonic() - frame_started)
            if wait > 0:
                self._stop_event.wait(wait)

    # ---------- 读取接口 ----------

    def latest(self):
        """最新样本 (monotonic, raw, value)；还没有样本返回 None。"""
        with self._lock:
            return self._samples[-1] if self._samples else None

    def sample_age(self, now=None):
        """最新样本的年龄（秒）；无样本返回 None。"""
        sample = self.latest()
        if sample is None:
            return None
        return (time.monotonic() if now is None else now) - sample[0]

    def snapshot(self):
        with self._lock:
            return list(self._samples)

    def save_window(self, path, center_time, before=2.0, after=2.0):
        """把 center_time ±[before, after] 秒内的样本另存一份 CSV（事件复盘用）。"""
        rows = [
            row for row in self.snapshot()
            if center_time - before <= row[0] <= center_time + after
        ]
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['monotonic', 'raw', 'value'])
            writer.writerows(rows)
        return len(rows)

    # ---------- 描述 ----------

    def describe(self):
        cfg = self.config
        if cfg['mode'] == 'tcp':
            link_desc = f"TCP {cfg['host']}:{cfg['tcp_port']}"
        else:
            link_desc = f"{cfg['port']} @ {cfg['baudrate']}"
        return (
            f"{link_desc} 从站={cfg['unit']} 寄存器=0x{int(cfg['register']):X}"
            f"×{cfg['count']} 高字在前={cfg['high_word_first']} scale={cfg['scale']}"
        )
