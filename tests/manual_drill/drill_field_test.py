# -*- coding: utf-8 -*-
"""现场钻孔测试脚本（DC 循环）—— 菜单版，备用工具。

用法（在仓库根目录下）：
    python tests\\manual_drill\\drill_field_test.py
    python tests\\manual_drill\\drill_field_test.py --sensor-port COM6 --sensor-baud 19200
    python tests\\manual_drill\\drill_field_test.py --serial-port COM7 --feed 500 --depth 2000 --lift 2000
    python tests\\manual_drill\\drill_field_test.py --dry          # 不接传感器，只试运动
    python tests\\manual_drill\\drill_field_test.py --list-ports   # 只列串口

现场建议流程：
    1. 手动把 XY 移到孔位、Z 到待钻高度（本脚本不会自动移动 XY）
    2. 菜单 s：确认传感器有读数（先解决 COM/波特率）
    3. 菜单 m：实时看 Z 与力值（不运动）
    4. 菜单 r：正式钻孔循环（需输入 RUN 确认）→ 自动停钻 + 抬升
    5. 结束看汇总；换头手动进行，电极下移参考 = A + Δ_head

安全：
    - 全程有人监护，手边可断电；菜单 e = 立即 STOP（固件同时关钻头）
    - 菜单 h = HOME（需输入 yes 确认）
    - 任何异常路径都是 STOP → 记录 → 抬升，不会继续下钻
"""

import argparse
import re
import sys
import time
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parents[1]        # tests/manual_drill -> 仓库根目录
PC_DIR = PROJECT_ROOT / 'code' / 'pc'
if str(PC_DIR) not in sys.path:
    sys.path.insert(0, str(PC_DIR))

from serial_motion import (BAUD, pick_port, open_verified_arduino, send,
                           read_response, query_positions, wait_positions,
                           normalize_motion_command)
from drill_workflows import SerialDrillIO
from force_sensor import ForceSensor, load_config as load_sensor_config
from drill_cycle import DrillParams, run_drill_cycle
from settings import DRILL_EVENT_LOG_DIR


class DummySensor:
    """dry 模式：恒定读数、永不触发，让循环靠安全限深中止后自动抬升。"""

    def __init__(self):
        self.total_samples = 0
        self.consecutive_failures = 0
        self.last_error = None

    def latest(self):
        self.total_samples += 1
        return (time.monotonic(), 0, 0.0)


def list_ports():
    from serial.tools import list_ports as serial_list_ports

    ports = list(serial_list_ports.comports())
    if not ports:
        print('没有检测到任何串口（检查 USB 线/驱动）')
        return
    print('检测到的串口：')
    for port in ports:
        print(f'  {port.device:<8} {port.description}')


def connect_sensor(args):
    config = load_sensor_config()
    if args.sensor_port:
        config['port'] = args.sensor_port
    if args.sensor_baud:
        config['baudrate'] = args.sensor_baud
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
            '检查 COM 口/波特率/接线，或先关掉 RS485 GUI'
        )
    sample = sensor.latest()
    print(f'传感器 OK：raw={sample[1]}  value={sample[2]}')
    print(f'数据记录到：{sensor.csv_path}')
    return sensor


def show_sensor_live(sensor, seconds=10.0):
    print(f'实时读数 {seconds:.0f} 秒（Ctrl+C 提前结束）：')
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            sample = sensor.latest()
            if sample is not None:
                print(
                    f'\r  raw={sample[1]:>10}  value={sample[2]:>12.3f}  '
                    f'连续失败={sensor.consecutive_failures}   ',
                    end='', flush=True,
                )
            else:
                print(
                    f'\r  （无数据）连续失败={sensor.consecutive_failures}  '
                    f'错误={sensor.last_error}   ',
                    end='', flush=True,
                )
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    print()


def show_monitor(ser, sensor, seconds=8.0):
    print(f'监控 Z 与力值 {seconds:.0f} 秒（不运动）：')
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        pos = query_positions(ser, timeout=1.0)
        z_text = pos['Z'] if pos else '?'
        if sensor is not None:
            sample = sensor.latest()
            force_text = f'raw={sample[1]} value={sample[2]:.3f}' if sample else '无数据'
        else:
            force_text = '未接传感器'
        print(f'\r  Z={z_text}  {force_text}   ', end='', flush=True)
        time.sleep(0.3)
    print()


def save_event_window(sensor, trigger):
    if sensor is None or not isinstance(trigger, dict) or trigger.get('time') is None:
        return
    stamp = time.strftime('%Y%m%d_%H%M%S')
    path = DRILL_EVENT_LOG_DIR / f'drill_event_{stamp}.csv'
    rows = sensor.save_window(path, float(trigger['time']), before=2.0, after=2.0)
    print(f'触发前后 {rows} 个传感器样本已保存：{path}')


def run_cycle(ser, sensor, params):
    dry = sensor is None
    label = 'dry 模式（不接传感器，钻到限深停）' if dry else sensor.describe()

    if not dry:
        sample = sensor.latest()
        if sample is None or (time.monotonic() - sample[0]) > 1.5:
            print('传感器没有新鲜读数，拒绝开始；先用 s 检查传感器')
            return

    print('\n=== 钻孔计划 ===')
    print(f'  进给速度：{params.feed_speed} 步/秒（{params.feed_speed / 100:.2f} mm/s）')
    print(f'  安全限深：{params.safe_depth_steps} 步（{params.safe_depth_steps / 100:.2f} mm）')
    print(f'  抬升距离：{params.lift_steps} 步（{params.lift_steps / 100:.2f} mm）')
    print(f'  总超时：  {params.timeout_seconds:.0f} 秒')
    print(f'  传感器：  {label}')
    print('  请确认：XY 已在孔位、Z 已在待钻高度、钻头周围无人、可随时断电。')
    if input('输入 RUN 开始（其它任意内容取消）：').strip() != 'RUN':
        print('已取消')
        return

    cycle_sensor = DummySensor() if dry else sensor
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

    io = SerialDrillIO(ser)
    try:
        result = run_drill_cycle(
            io, cycle_sensor, params=params, progress=progress, log=ui_log
        )
    except KeyboardInterrupt:
        print()
        print('用户中断：发送 STOP（固件同时关钻头）')
        send(ser, 'STOP')
        read_response(ser, markers=('STOP',), timeout=1.0)
        return

    print()
    pos = query_positions(ser)
    save_event_window(sensor, result.get('trigger'))
    print('=== 结果 ===')
    print(f"  status={result['status']}  lifted={result['lifted']}  "
          f"钻头停止已复核={result['drill_off_verified']}")
    print(f"  Z_start={result['z_start']}  Z_stop={result['z_stop']}  "
          f"钻深={result['depth_steps']} 步")
    print(f"  抬升目标 Z={result['z_lift_target']}  当前 POS={pos}")
    if result['status'] == 'BREAKTHROUGH' and result['lifted']:
        print('  完成：已停钻并抬升。换头请手动；电极下移参考 = A + Δ_head')
    else:
        print('  未正常完成：请人工确认机构状态后再操作')


def edit_params(params):
    print(f'当前参数：进给={params.feed_speed} 步/秒，'
          f'限深={params.safe_depth_steps} 步，抬升={params.lift_steps} 步，'
          f'超时={params.timeout_seconds:.0f} 秒')
    prompts = (
        ('feed_speed', '进给速度（步/秒）', float),
        ('safe_depth_steps', '安全限深（步）', int),
        ('lift_steps', '抬升距离（步）', int),
        ('timeout_seconds', '总超时（秒）', float),
    )
    for name, label, caster in prompts:
        text = input(f'  {label} [{getattr(params, name)}]（回车不改）：').strip()
        if text:
            try:
                setattr(params, name, caster(text))
            except ValueError:
                print(f'  输入无效，保持 {name}={getattr(params, name)}')


def manual_lift(ser, steps):
    pos = query_positions(ser)
    if pos is None:
        print('读取 POS 失败，未抬升')
        return
    target = pos['Z'] - steps
    send(ser, f'Z -{steps}')
    read_response(ser, timeout=0.5)
    ok = wait_positions(ser, {'Z': target}, timeout=30)
    print(f'抬升 {steps} 步：{"完成" if ok else "未确认"}（目标 Z={target}）')


def print_menu():
    print("""
================ 现场钻孔测试 ================
  s          传感器实时读数（10 秒）
  m          监控 Z 与力值（8 秒，不运动）
  z          查看当前 POS
  r          开始钻孔循环（需输入 RUN 确认）
  d          dry 试跑：不接传感器，钻到限深停 + 抬升
  l 500      手动抬升 500 步
  p          修改参数（进给/限深/抬升/超时）
  c          传感器连接失败后重连
  h          HOME 归零（需输入 yes 确认）
  e          立即 STOP（固件同时关钻头）
  q          退出（自动发送 E）
==============================================
""")


def main():
    parser = argparse.ArgumentParser(description='现场钻孔测试（DC 循环）')
    parser.add_argument('--serial-port', help='Arduino 串口（默认自动识别/交互选择）')
    parser.add_argument('--sensor-port', help='传感器 RS485 串口（默认读 pc/force_sensor.json）')
    parser.add_argument('--sensor-baud', type=int, help='传感器波特率（默认 19200）')
    parser.add_argument('--feed', type=float, default=500.0, help='进给速度 步/秒（默认 500 = 5mm/s）')
    parser.add_argument('--depth', type=int, default=2000, help='安全限深 步（默认 2000 = 20mm）')
    parser.add_argument('--lift', type=int, default=2000, help='抬升距离 步（默认 2000 = 20mm）')
    parser.add_argument('--timeout', type=float, default=60.0, help='总超时 秒（默认 60）')
    parser.add_argument('--dry', action='store_true', help='不接传感器：只试运动（到限深停）')
    parser.add_argument('--list-ports', action='store_true', help='只列出串口并退出')
    args = parser.parse_args()

    if args.list_ports:
        list_ports()
        return

    params = DrillParams(
        feed_speed=args.feed,
        safe_depth_steps=args.depth,
        lift_steps=args.lift,
        timeout_seconds=args.timeout,
    )

    list_ports()
    port = args.serial_port or pick_port()
    try:
        ser = open_verified_arduino(port)
    except RuntimeError as exc:
        print(f'Arduino 连接失败：{exc}')
        return
    print(f'Arduino 已连接 {port} @ {BAUD}')

    sensor = None
    if args.dry:
        print('dry 模式：不连接传感器')
    else:
        try:
            sensor = connect_sensor(args)
        except Exception as exc:
            print(f'传感器连接失败：{exc}')
            print('  可以先在菜单里用 c 重连；或加 --dry 只试运动。')

    print_menu()
    try:
        while True:
            try:
                line = input('drill> ').strip()
            except (EOFError, KeyboardInterrupt):
                break
            low = line.lower()
            if not low:
                continue
            if low in ('q', 'quit', 'exit'):
                break
            if low in ('s', 'sensor'):
                if sensor is None:
                    print('传感器未连接（用 c 重连）')
                else:
                    show_sensor_live(sensor)
            elif low in ('m', 'monitor'):
                show_monitor(ser, sensor)
            elif low in ('z', 'pos'):
                print(query_positions(ser))
            elif low in ('r', 'run'):
                run_cycle(ser, sensor, params)
            elif low in ('d', 'dry'):
                run_cycle(ser, None, params)
            elif low in ('p', 'params'):
                edit_params(params)
            elif low in ('c', 'connect'):
                if sensor is not None:
                    sensor.close()
                    sensor = None
                try:
                    sensor = connect_sensor(args)
                except Exception as exc:
                    print(f'传感器连接失败：{exc}')
            elif low in ('h', 'home'):
                if input('确认 HOME？（输入 yes）：').strip().lower() == 'yes':
                    send(ser, 'HOME')
                    out = read_response(
                        ser, markers=('HOME DONE', 'HOME FAIL'), timeout=120
                    )
                    print(out, end='')
                else:
                    print('已取消')
            elif low in ('e', 'stop'):
                send(ser, 'STOP')
                print(read_response(ser, timeout=1.0), end='')
            elif re.fullmatch(r'l\s*\d+', low):
                steps = int(re.sub(r'[^0-9]', '', low))
                manual_lift(ser, steps)
            else:
                print_menu()
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
