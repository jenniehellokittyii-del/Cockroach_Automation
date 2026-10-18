# -*- coding: utf-8 -*-
"""现场钻孔控制台 —— 手动控制位置 + RUN 开始钻孔（手感类似 motion_client）。

用法（在仓库根目录）：
    python tests\\manual_drill\\drill_console.py
    python tests\\manual_drill\\drill_console.py --dry      # 不接传感器，钻到限深自动停
    python tests\\manual_drill\\drill_console.py --sensor-port COM6 --feed 500 --depth 2000

现场流程：
    1. 连接 Arduino（校验固件 + 发 E）和传感器（默认 COM6 / 19200）
    2. 进入 drill> 控制台：手动移动 / 看力值 / 接近 / 急停
    3. 输入 RUN → 按当前参数执行钻孔循环；结束后回到控制台
    4. Q 退出（自动发 E）

安全：
    - 急停随时输入 E（固件同时关钻头）
    - 接近命令 A 不转钻头；只有 RUN 才会 DRILL ON
    - 钻孔过程中按 Ctrl+C 会中止并发送 STOP
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

# ==================== 现场参数（只改这里） ====================
SENSOR_PORT = 'COM6'        # 传感器 COM（2026-10-17 现场实测：COM6 = FTDI USB-RS485）
SENSOR_BAUD = 19200         # 已确认
FEED_STEPS_PER_SEC = 500    # 进给：500 步/秒 = 5 mm/s（与手动 Z 速度一致，现场确认）
SAFE_DEPTH_STEPS = 2000     # 安全限深：2000 步 = 20 mm（现场临时放宽；按实际间隙调整）
LIFT_STEPS = 2000           # 触发后抬升：2000 步 = 20 mm（现场确认）
TIMEOUT_SECONDS = 60        # 总超时
APPROACH_STEPS = 500        # 接近命令 A 的默认步数（现场暂不用 A，可忽略）
STEP_SIGMA_K = 2.5          # 小信号模式：回落阈值 2.5×噪声（再灵敏一档）
STEP_LOAD_K = 1.5           # 加载判定：峰值高出基线 1.5×噪声
STEP_MIN_NOISE = 0.5        # 噪声下限（传感器只有 ±1~2 跳变）
STEP_MIN_DROP = 2.0         # 最小回落（原始值单位，防 1~2 单位噪声）
STEP_CONFIRM = 1            # 1 个样本立即触发（信号已确认干净；误触发再调回 2）
EXTRA_DRILL_SECONDS = 0.0   # 检测后不再继续下钻（不停留）
MODE = 'real'               # 'real' = 用传感器；'dry' = 不接传感器，钻到限深自动停
# ==============================================================


class ConsoleDrillIO(SerialDrillIO):
    """控制台专用 IO：把 ZDOWN 换成相对下行（Z 100000），负坐标也不会反向。"""

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
    """dry 模式：恒定读数、永不触发，靠安全限深中止后自动抬升。"""

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
        return []
    print('检测到的串口：')
    for port in ports:
        print(f'  {port.device:<8} {port.description}')
    return ports


def resolve_sensor_port(cli_port):
    """优先级：命令行 --sensor-port > 顶部 SENSOR_PORT > 交互选择。"""
    port = cli_port or SENSOR_PORT
    if port:
        return port
    ports = list_ports()
    if not ports:
        return ''
    return input('输入传感器 COM 口号（如 COM6，回车退出）：').strip()


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
            '检查 COM/波特率/接线，或先关掉 RS485 GUI'
        )
    sample = sensor.latest()
    print(f'传感器 OK：raw={sample[1]}  value={sample[2]}')
    print(f'数据记录到：{sensor.csv_path}')
    return sensor


def show_force_live(sensor, seconds=10.0):
    values = []
    end = time.monotonic() + seconds
    try:
        while time.monotonic() < end:
            sample = sensor.latest()
            if sample is not None:
                values.append(sample[1])
                print(f'\r  力值 raw={sample[1]:>10}  value={sample[2]:>12.3f}   ',
                      end='', flush=True)
            else:
                print(f'\r  （无数据）错误={sensor.last_error}   ', end='', flush=True)
            time.sleep(0.15)
    except KeyboardInterrupt:
        pass
    print()
    if values:
        print(f'  本段最小 {min(values)} / 最大 {max(values)} / 变化量 {max(values) - min(values)}')


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


def edit_params(params):
    print(f'当前参数：进给={params.feed_speed:g} 步/秒，'
          f'限深={params.safe_depth_steps} 步，抬升={params.lift_steps} 步，'
          f'超时={params.timeout_seconds:g} 秒')
    prompts = (
        ('feed_speed', '进给速度（步/秒）', float),
        ('safe_depth_steps', '安全限深（步）', int),
        ('lift_steps', '抬升距离（步）', int),
        ('timeout_seconds', '总超时（秒）', float),
        ('extra_drill_seconds', '检测后继续下钻（秒，0.2≈1mm）', float),
    )
    for name, label, caster in prompts:
        text = input(f'  {label} [{getattr(params, name)}]（回车不改）：').strip()
        if text:
            try:
                setattr(params, name, caster(text))
            except ValueError:
                print(f'  输入无效，保持 {name}={getattr(params, name)}')
    print('  检测参数（小信号模式）：' + str(params.step_detect_kwargs))
    detector_prompts = (
        ('sigma_k', '回落阈值倍数 sigma_k', float),
        ('load_k', '加载判定倍数 load_k', float),
        ('min_noise', '噪声下限 min_noise', float),
        ('min_drop', '最小回落 min_drop', float),
        ('confirm_samples', '连续确认样本数', int),
    )
    for key, label, caster in detector_prompts:
        current = params.step_detect_kwargs.get(key)
        text = input(f'  {label} [{current}]（回车不改）：').strip()
        if text:
            try:
                params.step_detect_kwargs[key] = caster(text)
            except ValueError:
                print(f'  输入无效，保持 {key}={current}')


def approach(ser, steps):
    """接近：快速下降 N 步，不转钻头（不检测接触）。"""
    pos = query_positions(ser)
    if pos is None:
        print('读取 POS 失败，未接近')
        return
    print(f'接近：从当前 Z={pos["Z"]} 快速下降 {steps} 步（{steps / 100:.2f} mm），钻头不转。')
    print('⚠ 不检测接触：下降距离必须小于「钻头到材料」的实际间隙！')
    if input('确认接近？(yes)：').strip().lower() != 'yes':
        print('已取消')
        return
    send(ser, 'SPEED 500')
    read_response(ser, timeout=0.5)
    send(ser, f'Z {steps}')
    read_response(ser, timeout=0.5)
    target = pos['Z'] + steps
    ok = wait_positions(ser, {'Z': target}, timeout=30)
    after = query_positions(ser)
    print(f'接近{"完成" if ok else "未确认"}：目标 Z={target}，当前 POS={after}')
    if not ok:
        print('接近未确认（可能到限位/运动故障），已发送 E 停止')
        send(ser, 'E')
        read_response(ser, timeout=0.5)


def run_cycle(ser, sensor, params):
    """按当前参数执行一次钻孔循环；结束后返回控制台。sensor=None 为 dry。"""
    dry = sensor is None
    if not dry:
        sample = sensor.latest()
        if sample is None or (time.monotonic() - sample[0]) > 1.5:
            print('传感器没有新鲜读数，拒绝开始；先按 F 检查')
            return

    print('\n=== 钻孔计划 ===')
    print(f'  进给：{params.feed_speed:g} 步/秒（{params.feed_speed / 100:.2f} mm/s）'
          f'｜限深：{params.safe_depth_steps} 步（{params.safe_depth_steps / 100:.2f} mm）'
          f'｜抬升：{params.lift_steps} 步')
    print(f'  限深从当前 Z 向下算；传感器：{"dry（不接传感器）" if dry else sensor.describe()}')
    print('  急停：钻孔中按 Ctrl+C（会发送 STOP）')

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

    io = ConsoleDrillIO(ser)
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
    pos = query_positions(ser)
    trigger = result.get('trigger')
    if sensor is not None and isinstance(trigger, dict) and trigger.get('time') is not None:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        path = DRILL_EVENT_LOG_DIR / f'drill_event_{stamp}.csv'
        rows = sensor.save_window(path, float(trigger['time']), before=2.0, after=2.0)
        print(f'触发前后 {rows} 个力值样本已保存：{path}')
    print('=== 结果 ===')
    print(f"  status={result['status']}  lifted={result['lifted']}  "
          f"钻头停止已复核={result['drill_off_verified']}")
    print(f"  Z_start={result['z_start']}  Z_stop={result['z_stop']}  "
          f"钻深={result['depth_steps']} 步")
    print(f"  抬升目标 Z={result['z_lift_target']}  当前 POS={pos}")
    if result['status'] == 'BREAKTHROUGH' and result['lifted']:
        print('  完成：已停钻并抬升。换头请手动：电极下移参考 = A + Δ_head')
    else:
        print('  未正常完成：请人工确认机构状态后再操作')
    depth = result.get('depth_steps')
    if isinstance(depth, int) and depth > 0:
        print(f'  提示：本次实际钻深 {depth} 步（{depth / 100:.2f} mm）。'
              f'如为正常刺穿，下次可用 CFG 把限深调到约 {depth + 100} 步。')
    if result['status'] == 'DEPTH_LIMIT':
        print(f'  本次先到限深（{params.safe_depth_steps} 步）才停：'
              '说明没检测到阶跃，或起始位置离材料太远/限深太小。')
    print('（已回到控制台）')


def send_manual(ser, line):
    """把手动指令发给 Arduino，行为与 motion_client 一致。"""
    command = normalize_motion_command(line)
    up = command.upper()
    send(ser, command)
    home_markers = {
        'HOME': ('HOME DONE', 'FAIL'),
        'HX': ('HOMED X', 'FAIL'),
        'HY': ('HOMED Y', 'FAIL'),
        'HZ': ('HOMED Z', 'FAIL'),
    }
    if up in home_markers:
        out = read_response(ser, markers=home_markers[up], timeout=120)
    else:
        out = read_response(ser, timeout=0.3)
    print(out, end='')


def print_console_help():
    print(f"""
================ 现场钻孔控制台 ================
【手动移动】(与 motion_client 相同)
  X100 / Y-50 / Z200 / B10        相对移动 N 步（Z+ 向下）
  G0 X1000 Y2000 Z500             绝对步数移动
  M0 X50 Y30 Z10                  绝对毫米移动
  POS (P) / SPEED500 / HOME / HX / HY / HZ
  DRILL ON / OFF / STATUS / PWM 50
  E / STOP                        急停（固件同时关钻头）
【接近】
  A                               快速下降 {APPROACH_STEPS} 步（不转钻头，需 yes 确认）
  A 1500                          快速下降 1500 步
【钻孔】
  RUN                             按当前参数开始钻孔（负 Z 也可用，内部相对下行）
  CFG                             修改 进给/限深/抬升/超时
【传感器】
  F  单次读数      S  实时 5 秒      M  监控 Z+力值 8 秒
【其它】
  ?  帮助          Q  退出（自动发 E）
================================================
""")


def main():
    parser = argparse.ArgumentParser(description='现场钻孔控制台（手动控制 + RUN）')
    parser.add_argument('--serial-port', help='Arduino 串口（默认自动识别/交互选择）')
    parser.add_argument('--sensor-port', help=f'传感器串口（默认 {SENSOR_PORT}）')
    parser.add_argument('--baud', type=int, help=f'传感器波特率（默认 {SENSOR_BAUD}）')
    parser.add_argument('--feed', type=float, help=f'进给步/秒（默认 {FEED_STEPS_PER_SEC}）')
    parser.add_argument('--depth', type=int, help=f'安全限深步（默认 {SAFE_DEPTH_STEPS}）')
    parser.add_argument('--lift', type=int, help=f'抬升步（默认 {LIFT_STEPS}）')
    parser.add_argument('--timeout', type=float, help=f'总超时秒（默认 {TIMEOUT_SECONDS}）')
    parser.add_argument('--dry', action='store_true', help='不接传感器（覆盖 MODE）')
    args = parser.parse_args()

    params = DrillParams(
        feed_speed=args.feed if args.feed is not None else FEED_STEPS_PER_SEC,
        safe_depth_steps=args.depth if args.depth is not None else SAFE_DEPTH_STEPS,
        lift_steps=args.lift if args.lift is not None else LIFT_STEPS,
        timeout_seconds=args.timeout if args.timeout is not None else TIMEOUT_SECONDS,
        step_detect_kwargs={
            'sigma_k': STEP_SIGMA_K,
            'load_k': STEP_LOAD_K,
            'min_noise': STEP_MIN_NOISE,
            'min_drop': STEP_MIN_DROP,
            'confirm_samples': STEP_CONFIRM,
        },
        extra_drill_seconds=EXTRA_DRILL_SECONDS,
        allow_negative_z=True,
    )
    dry = args.dry or MODE.lower() == 'dry'

    print('=' * 64)
    print('现场钻孔控制台')
    print(f'  模式：{"dry（不接传感器，钻到限深停）" if dry else "real（传感器阶跃停钻）"}'
          f'｜进给 {params.feed_speed:g} 步/秒（{params.feed_speed / 100:.2f} mm/s）'
          f'｜限深 {params.safe_depth_steps} 步（{params.safe_depth_steps / 100:.2f} mm）')
    print(f'  接近默认 {APPROACH_STEPS} 步（{APPROACH_STEPS / 100:.2f} mm）｜'
          f'抬升 {params.lift_steps} 步（{params.lift_steps / 100:.2f} mm）')
    print(f'  检测：sigma_k={STEP_SIGMA_K} load_k={STEP_LOAD_K} '
          f'min_noise={STEP_MIN_NOISE} min_drop={STEP_MIN_DROP} confirm={STEP_CONFIRM}')
    print('=' * 64)
    list_ports()

    port = args.serial_port or pick_port()
    try:
        ser = open_verified_arduino(port)
    except RuntimeError as exc:
        print(f'Arduino 连接失败：{exc}')
        return
    print(f'Arduino 已连接 {port} @ {BAUD}')

    sensor = None
    if dry:
        print('dry 模式：跳过传感器')
    else:
        baud = args.baud or SENSOR_BAUD
        sensor_port = resolve_sensor_port(args.sensor_port)
        for attempt in range(3):
            if not sensor_port:
                print('没有可用的传感器串口，退出')
                send(ser, 'E')
                ser.close()
                return
            try:
                sensor = connect_sensor(sensor_port, baud)
                break
            except Exception as exc:
                print(f'传感器连接失败：{exc}')
                if attempt == 2:
                    print('连续失败，退出；可先加 --dry 只试运动')
                    send(ser, 'E')
                    ser.close()
                    return
                list_ports()
                sensor_port = input('换一个传感器 COM 口（回车退出）：').strip()

    print_console_help()
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
            if up in ('?', 'HELP', 'H'):
                print_console_help()
                continue
            if up == 'RUN':
                run_cycle(ser, None if dry else sensor, params)
                continue
            if up == 'CFG' or up == 'PARAMS':
                edit_params(params)
                continue
            approach_match = re.fullmatch(r'A\s*(\d*)', up)
            if approach_match:
                steps = int(approach_match.group(1)) if approach_match.group(1) else APPROACH_STEPS
                approach(ser, steps)
                continue
            if up in ('F', 'FORCE'):
                if sensor is None:
                    print('未接传感器（dry 模式）')
                else:
                    print(sensor.latest())
                continue
            if up in ('S', 'SENSOR'):
                if sensor is None:
                    print('未接传感器（dry 模式）')
                else:
                    show_force_live(sensor)
                continue
            if up in ('M', 'MONITOR'):
                show_monitor(ser, sensor)
                continue
            try:
                send_manual(ser, line)
            except ValueError as exc:
                print(f'输入错误：{exc}')
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
