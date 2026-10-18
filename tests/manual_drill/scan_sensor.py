# -*- coding: utf-8 -*-
"""RS485 力传感器寄存器扫描（现场诊断用）。

用途：钻孔时力值一直不变，检查“力”是否被变送器放在其他寄存器。
流程：
    1. 读保持寄存器 0..31 与 0x9C40..0x9C4F（以及输入寄存器）做快照；
    2. 提示你用手反复按压传感器平台 → 回车开始 10 秒监视；
    3. 打印有变化的寄存器（地址 + 最小/最大值）。

用法（在仓库根目录；先退出正在运行的控制台脚本，避免串口被占用）：
    python tests\\manual_drill\\scan_sensor.py
    python tests\\manual_drill\\scan_sensor.py --port COM6 --baud 19200 --seconds 15
"""

import argparse
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]        # tests/manual_drill -> 仓库根目录
PC_DIR = PROJECT_ROOT / 'code' / 'pc'
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

from force_sensor import find_modbus_link_dir

BLOCKS = ((0x0000, 32), (0x9C40, 2))


def open_link(port, baud, unit, timeout=0.3):
    link_dir = find_modbus_link_dir()
    if link_dir is None:
        raise RuntimeError('找不到 modbus_link.py')
    if str(link_dir) not in sys.path:
        sys.path.insert(0, str(link_dir))
    import modbus_link as mb
    link = mb.ModbusLink(mode='serial', port=port, baudrate=baud, unit=unit, timeout=timeout)
    link.open()
    return link


def snapshot(link):
    for func in (3, 4):
        print(f'== 寄存器快照 func={func:02d}（只列非零）==')
        for base, count in BLOCKS:
            try:
                vals = link.read_registers(base, count, func=func)
            except Exception as exc:
                print(f'  0x{base:04X}..0x{base + count - 1:04X}  读取失败: {exc}')
                continue
            for index, value in enumerate(vals):
                if value:
                    print(f'  0x{base + index:04X} = {value} (0x{value:04X})')
        if func == 3:
            # 额外逐个探测 0x9C42..0x9C4F，排除“力在相邻寄存器”的可能
            for addr in range(0x9C42, 0x9C50):
                try:
                    value = link.read_registers(addr, 1, func=func)[0]
                except Exception:
                    continue
                if value:
                    print(f'  0x{addr:04X} = {value} (0x{value:04X})')


def monitor(link, seconds):
    stats = {}
    started = time.monotonic()
    cycles = 0
    while time.monotonic() - started < seconds:
        for func in (3, 4):
            for base, count in BLOCKS:
                try:
                    vals = link.read_registers(base, count, func=func)
                except Exception:
                    continue
                for index, value in enumerate(vals):
                    key = (func, base + index)
                    low, high = stats.get(key, (value, value))
                    stats[key] = (min(low, value), max(high, value))
        cycles += 1
        time.sleep(0.05)
    print(f'监视结束：共 {cycles} 轮，{seconds:.0f} 秒')

    changed = {
        key: (low, high) for key, (low, high) in stats.items() if high - low > 0
    }
    if not changed:
        print('❌ 所有扫描的寄存器在监视期间都没有变化。')
        print('   → 按压产生的力没有进入变送器：检查称重传感器是否在受力路径上、')
        print('     四线接线（E+/E-/S+/S-）以及变送器量程/零点。')
        return
    print('✅ 有变化的寄存器：')
    for (func, addr), (low, high) in sorted(changed.items()):
        print(f'  func={func:02d} 0x{addr:04X}: {low} ~ {high}  (变化 {high - low})')
    for (func, addr), (low, high) in sorted(changed.items()):
        if func == 3 and addr in (0x9C40, 0x9C41):
            print(f'  注意：0x9C40 窗口有变化，说明力确实走这个寄存器（此前可能是幅度太小/安装问题）。')


def main():
    parser = argparse.ArgumentParser(description='RS485 力传感器寄存器扫描（现场诊断）')
    parser.add_argument('--port', default='COM6')
    parser.add_argument('--baud', type=int, default=19200)
    parser.add_argument('--unit', type=int, default=1)
    parser.add_argument('--seconds', type=float, default=10.0)
    args = parser.parse_args()

    link = open_link(args.port, args.baud, args.unit)
    try:
        snapshot(link)
        print()
        print(f'接下来 {args.seconds:.0f} 秒请用手反复按压「传感器平台/样品台」。')
        input('准备好后按回车开始监视...')
        monitor(link, args.seconds)
    finally:
        link.close()


if __name__ == '__main__':
    main()
