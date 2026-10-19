# -*- coding: utf-8 -*-
"""钻孔现场操作台（精简版）。

用法（在仓库根目录）：
    python code/pc/drill_console.py
    python code/pc/drill_console.py --sensor-port COM6 --feed 500 --depth 2000 --lift 2000
    python code/pc/drill_console.py --dry      # 不接传感器，只验证运动/停/抬升

操作：
    X100 / Y-50 / Z200 / B10   相对移动（Z+ 向下）
    G0 X1000 Y2000 Z500        绝对步数
    M0 X50 Y30 Z10             绝对毫米
    POS                        查询坐标
    ZERO                       坐标清零（不移动机械）
    A [N]                      快速下降（不转钻头，默认 500 步）
    RUN                        开始钻孔（阶跃停钻 + 安全限深 + 抬升）
    F                          读一次力值
    E                          急停（固件同时关钻头）
    Q                          退出（自动发 E）

安全：全程有人监护；RUN 前确认 XY 在孔位、Z 在待钻高度。
"""

import argparse
import re
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from serial_motion import (BAUD, pick_port, open_verified_arduino, send,
                           read_response, query_positions, wait_positions,
                           normalize_motion_command)
from drill_workflows import SerialDrillIO
from force_sensor import ForceSensor, load_config as load_sensor_config
from drill_cycle import DrillParams, run_drill_cycle
from settings import DRILL_EVENT_LOG_DIR

# ==================== 现场参数（只改这里） ====================
SENSOR_PORT = 'COM6'        # 传感器 COM（现场实测 COM6 = FTDI USB-RS485）
SENSOR_BAUD = 19200
FEED_STEPS_PER_SEC = 500    # 进给 500 步/秒 = 5 mm/s
SAFE_DEPTH_STEPS = 2000     # 安全限深 2000 步 = 20 mm
LIFT_STEPS = 2000           # 触发后抬升 2000 步 = 20 mm
TIMEOUT_SECONDS = 60
APPROACH_STEPS = 500        # A 命令默认下降步数
# ==============================================================


class ConsoleDrillIO(SerialDrillIO):
    """控制台专用：ZDOWN 换成相对下行，负坐标也不会反向。"""

    REL_DOWN_STEPS = 100000

    def __init__(self, ser):
        super().__init__(ser)
        self._expect_down_ack = False

    def send(self, line):
        if line.strip().upper() in ('ZDOWN', 'ZD'):
            self._expect_down_ack = True
            line = f'Z {self.REL_DOWN_STEPS}'
        super().send(line)

    def read_response(self, markers=None, timeout=0.3):
        out = super().read_response(markers=markers, timeout=timeout)
        if self._expect_down_ack:
            self._expect_down_ack = False
            marker = f'MOVE Z {self.REL_DOWN_STEPS}'
            if marker in out:
                out = out.replace(marker, 'MOVE Z DOWN CONTINUOUS (REL)')
        return out


class DummySensor:
    """dry 模式：永不触发，靠限深中止后抬升。"""

    def __init__(self):
        self.total_samples = 0
        self.consecutive_failures = 0
        self.last_error = None

    def latest(self):
        self.total_samples += 1
        return (time.monotonic(), 0, 0.0)


def connect_sensor(port, baud):
    config = load_sensor_config()
    config['port'] = port
    config['baudrate'] = int(baud)
    sensor = ForceSensor(config=config)
    print(f'连接传感器：{sensor.describe()}')
    sensor.start()
    deadline = time.monotonic() + 3.0
    while sensor.latest() is None and time.monotonic() < deadline:
        time.sleep(0.05)
    if sensor.latest() is None:
        sensor.close()
        raise RuntimeError(
            f'传感器 3 秒内没有数据（{sensor.last_error}）；'
            '检查 COM/波特率/接线，或先关闭 RS485 GUI'
        )
    sample = sensor.latest()
    print(f'传感器 OK：raw={sample[1]}  value={sample[2]}')
    print(f'数据记录到：{sensor.csv_path}')
    return sensor


def run_cycle(ser, sensor, params):
    dry = sensor is None
    if not dry:
        sample = sensor.latest()
        if sample is None or time.monotonic() - sample[0] > 1.5:
            print('传感器无新鲜读数，拒绝开始（按 F 检查）')
            return

    print('\n=== 钻孔计划 ===')
    print(f'  进给 {params.feed_speed:g} 步/秒｜限深 {params.safe_depth_steps} 步'
          f'｜抬升 {params.lift_steps} 步')
    print(f'  传感器：{sensor.describe() if not dry else "dry（不接传感器）"}')
    print('  确认：XY 在孔位、Z 在待钻高度、周围安全。输入 RUN 开始。')
    last_print = [0.0]

    def progress(info):
        now = time.monotonic()
        if now - last_print[0] < 0.5:
            return
        last_print[0] = now
        sample = info.get('sample')
        force_text = f'raw={sample[1]} value={sample[2]:.3f}' if sample else '无数据'
        depth = info.get('depth')
        depth_text = f'{depth} 步' if depth is not None else '-'
        print(
            f"\r[钻孔] t={info['elapsed']:5.1f}s  Z={info.get('z')}  "
            f"深={depth_text}/{params.safe_depth_steps}  {force_text}    ",
            end='', flush=True,
        )

    def ui_log(message):
        print()
        print(message)

    io = ConsoleDrillIO(ser)
    cycle_sensor = DummySensor() if dry else sensor
    try:
        result = run_drill_cycle(
            io, cycle_sensor, params=params, progress=progress, log=ui_log
        )
    except KeyboardInterrupt:
        print('\n用户中断：发送 STOP')
        send(ser, 'STOP')
        read_response(ser, markers=('STOP',), timeout=1.0)
        return

    print()
    trigger = result.get('trigger')
    if sensor is not None and isinstance(trigger, dict) and trigger.get('time') is not None:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = DRILL_EVENT_LOG_DIR / f'drill_event_{stamp}.csv'
        rows = sensor.save_window(path, float(trigger['time']), before=2.0, after=2.0)
        print(f'触发前后 {rows} 个力值样本已保存：{path}')
    print(f"结果：status={result['status']}  lifted={result['lifted']}  "
          f"钻深={result['depth_steps']} 步  Z_stop={result['z_stop']}")
    if result['status'] == 'BREAKTHROUGH' and result['lifted']:
        print('完成：已停钻并抬升。换头请手动；电极下移参考 = A + Δ_head')
    else:
        print('未正常完成：请人工确认机构状态后再操作')


def main():
    parser = argparse.ArgumentParser(description='钻孔现场操作台（精简版）')
    parser.add_argument('--serial-port', help='Arduino 串口（默认自动识别）')
    parser.add_argument('--sensor-port', default=SENSOR_PORT, help=f'传感器串口（默认 {SENSOR_PORT}）')
    parser.add_argument('--baud', type=int, default=SENSOR_BAUD, help=f'传感器波特率（默认 {SENSOR_BAUD}）')
    parser.add_argument('--feed', type=float, default=FEED_STEPS_PER_SEC, help=f'进给步/秒（默认 {FEED_STEPS_PER_SEC}）')
    parser.add_argument('--depth', type=int, default=SAFE_DEPTH_STEPS, help=f'安全限深步（默认 {SAFE_DEPTH_STEPS}）')
    parser.add_argument('--lift', type=int, default=LIFT_STEPS, help=f'抬升步（默认 {LIFT_STEPS}）')
    parser.add_argument('--timeout', type=float, default=TIMEOUT_SECONDS, help=f'总超时秒（默认 {TIMEOUT_SECONDS}）')
    parser.add_argument('--dry', action='store_true', help='不接传感器，只验证运动/停/抬升')
    args = parser.parse_args()

    params = DrillParams(
        feed_speed=args.feed,
        safe_depth_steps=args.depth,
        lift_steps=args.lift,
        timeout_seconds=args.timeout,
        allow_negative_z=True,   # 控制台内部用相对下行
    )

    port = args.serial_port or pick_port()
    try:
        ser = open_verified_arduino(port)
    except RuntimeError as exc:
        print(f'Arduino 连接失败：{exc}')
        return
    print(f'Arduino 已连接 {port} @ {BAUD}')

    sensor = None
    if args.dry:
        print('dry 模式：跳过传感器')
    else:
        try:
            sensor = connect_sensor(args.sensor_port, args.baud)
        except Exception as exc:
            print(f'传感器连接失败：{exc}')
            print('  可用 --dry 只试运动；或检查 config/force_sensor.json')
            ser.close()
            return

    print("""
命令：X100 / Y-50 / Z200 / B10 相对移动；G0/M0 绝对移动；POS；ZERO
      A [N] 快速下降（不转钻头）；RUN 钻孔；F 读力值；E 急停；Q 退出
""")
    try:
        while True:
            try:
                line = input('drill> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            if not line:
                continue
            up = line.upper()
            if up in ('Q', 'QUIT', 'EXIT'):
                break
            if up == '?':
                print('X/Y/Z/B、G0、M0、POS、ZERO、A [N]、RUN、F、E、Q')
                continue
            if up == 'RUN':
                run_cycle(ser, None if args.dry else sensor, params)
                continue
            if up == 'F':
                if sensor is None:
                    print('dry 模式：未接传感器')
                else:
                    print(sensor.latest())
                continue
            if up in ('E', 'STOP'):
                send(ser, 'STOP')
                print(read_response(ser, timeout=1.0), end='')
                continue
            match = re.fullmatch(r'A\s*(\d*)', up)
            if match:
                steps = int(match.group(1)) if match.group(1) else APPROACH_STEPS
                pos = query_positions(ser)
                if pos is None:
                    print('读取 POS 失败，未接近')
                    continue
                print(f'接近：Z={pos["Z"]} → 下降 {steps} 步（{steps / 100:.2f} mm），钻头不转')
                if input('确认？(yes)：').strip().lower() != 'yes':
                    print('已取消')
                    continue
                send(ser, 'SPEED 500')
                read_response(ser, timeout=0.5)
                send(ser, f'Z {steps}')
                read_response(ser, timeout=0.5)
                ok = wait_positions(ser, {'Z': pos['Z'] + steps}, timeout=30)
                print(f'接近{"完成" if ok else "未确认"}，POS={query_positions(ser)}')
                continue
            try:
                command = normalize_motion_command(line)
            except ValueError as exc:
                print(f'输入错误：{exc}')
                continue
            if not (
                re.fullmatch(r'[XYZB]\s*-?\d+', command.upper())
                or command.upper() in ('POS', 'P', 'ZERO', 'R')
                or command.upper().startswith(('G0', 'M0'))
            ):
                print('不支持的命令（? 查看帮助）')
                continue
            send(ser, command)
            print(read_response(ser, timeout=0.5), end='')
    finally:
        try:
            send(ser, 'E')
            out = read_response(ser, timeout=0.5)
            if out:
                print(out, end='')
        except Exception:
            pass
        if sensor is not None:
            sensor.close()
        ser.close()
        print('已退出（E 已发送）')


if __name__ == '__main__':
    main()
