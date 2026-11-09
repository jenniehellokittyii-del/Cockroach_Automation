# -*- coding: utf-8 -*-
"""启动归零 → 选择供料口取电极 → 头部选点 → 安全高度 → 确认 → D1–D3。

只使用一个 Arduino 连接；确认后连接 USB-CAN 转盘并复用 D1–D3。
供料动作沿用 feed_port_positioner 的绝对坐标定位。
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import re
import subprocess
import sys
from pathlib import Path

import serial_motion as motion
from feed_port_positioner import (
    FEED_PORTS, MotionController, SAFE_LIFT_STEPS, move_to_feed_port,
    validate_feed_ports_after_home,
)
from vision_workflows import run_visual_head
from settings import TURNTABLE_PORT, TURNTABLE_BAUD, VISION_POS_STEPS
from turntable import Turntable
import drill_workflows as wf


POSITIONING_SPEED = 650  # 步/秒；仅用于取料和视觉定位，原速度为 500。


def choose_feed_port():
    while True:
        answer = input('要取哪一个供料口？输入 1–12 或 P1–P12，Q 取消: ').strip().upper()
        if answer in ('Q', 'QUIT', 'EXIT', 'N', ''):
            return None
        number = answer[1:] if answer.startswith('P') else answer
        if number.isascii() and number.isdigit() and int(number) in FEED_PORTS:
            return int(number)
        print('请输入 1–12 或 P1–P12，Q 取消。')


def set_motion_speed(controller, speed):
    controller.send(f'SPEED {speed:g}')
    response = controller.read_response(timeout=0.3)
    values = re.findall(r'^SPEED=([0-9]+(?:\.[0-9]+)?)\s*$', response, re.MULTILINE)
    if controller.fault or 'ERR' in response or not values or float(values[-1]) != speed:
        motion.vision_reference_ready = False
        controller.emergency_stop()
        raise RuntimeError(f'速度 {speed:g} 步/秒未获确认，流程停止：{response.strip()}')


@contextmanager
def positioning_speed(controller):
    normal_speed = wf.load_params().normal_speed
    try:
        set_motion_speed(controller, POSITIONING_SPEED)
        yield
    except BaseException:
        motion.vision_reference_ready = False
        controller.emergency_stop()
        raise
    finally:
        set_motion_speed(controller, normal_speed)


def confirm(prompt):
    while True:
        answer = input(prompt + ' (y/N): ').strip().lower()
        if answer in ('y', 'yes'):
            return True
        if answer in ('', 'n', 'no', 'q', 'quit', 'exit'):
            return False
        print('请输入 Y 开始，N 或 Q 取消。')


def home(controller):
    motion.vision_reference_ready = False
    wf.LAST_DRILL_RESULT = None
    if not controller.home_all():
        return False
    if not validate_feed_ports_after_home(controller.home_position):
        return False
    motion.motion_fault = None
    motion.vision_reference_ready = True
    return True


def pickup_electrode(controller, port_number):
    if port_number not in FEED_PORTS:
        raise ValueError('供料口编号必须是 1～12')
    if not motion.vision_reference_ready or motion.motion_fault:
        print('尚未成功归零或存在运动故障，禁止取料。')
        return False
    with positioning_speed(controller):
        return _pickup_electrode(controller, port_number)


def _pickup_electrode(controller, port_number):
    if not move_to_feed_port(controller, port_number):
        motion.vision_reference_ready = False
        return False
    position = controller.query_position()
    if position is None or controller.fault:
        controller.emergency_stop()
        motion.vision_reference_ready = False
        return False
    safe_z = max(position['Z'] - SAFE_LIFT_STEPS, controller.home_position['Z'])
    print(f'{port_number}号供料口定位完成，Z 抬升 {SAFE_LIFT_STEPS} 步到 {safe_z}')
    if not controller.move_absolute({'Z': safe_z}):
        motion.vision_reference_ready = False
        return False
    print(f'{port_number}号取料定位和抬升完成。')
    return True


def visual_position(controller):
    # HOME 已在启动阶段完成；复用视觉位的“抬Z → XY → 下Z”绝对移动。
    print('自动化视觉：头部识别 VIS_HEAD，选择 P1–P4 中的一个点。')
    with positioning_speed(controller):
        return run_visual_head(
            controller.ser, skip_home=True, single_point=True,
            retry_capture=lambda: confirm('识别未完成，是否重新识别？'),
        )


def drill_after_selection(controller):
    """沿用已选点 XY，绝对抬升至 SAFE，确认后执行 TZ/TE/FULL。"""
    wf.LAST_DRILL_RESULT = None
    state = wf.load_state()
    position = motion.query_positions(controller.ser)
    if (not motion.vision_reference_ready or motion.motion_fault or position is None
            or position['X'] != position['X2'] or position['Z'] != VISION_POS_STEPS['Z']):
        print(f'选点后位置或运动状态异常，未开始钻孔：{position}')
        return False
    safe = state['safe_z_steps']
    if not controller.home_position['Z'] <= safe < position['Z']:
        print(f'SAFE={safe} 不在归零高度与视觉高度之间，禁止换头。')
        return False
    if state['drill_start_z'] < safe:
        print('DRILLSTART 位于 SAFE 上方，请核对参数，未开始钻孔。')
        return False
    print(f'选点完成，保持 XY，Z={position["Z"]} → SAFE={safe}')
    if not motion.move_continuous(controller.ser, {'Z': safe}):
        return False
    print(f'本轮参数：SAFE={safe} DRILLSTART={state["drill_start_z"]} '
          f'OFFSET={state["offset_steps"]} INSERT={state["insert_steps"]} '
          f'drill_duty={wf.load_params().drill_duty}')
    if not confirm('确认当前电极头位于工具1，开始 TZ → TE → D1 → D2 → D3？'):
        print('已取消钻孔，保持在 SAFE。')
        return True
    # 用户确认期间仍需复核位置，防止使用已变动的选点位置。
    current = motion.query_positions(controller.ser)
    if (motion.motion_fault or current is None or current['X2'] != position['X']
            or current['X'] != position['X'] or current['Y'] != position['Y']
            or current['Z'] != safe):
        print('确认后位置异常，未连接转盘或开始钻孔。')
        return False
    tt = None
    try:
        tt = Turntable(TURNTABLE_PORT, TURNTABLE_BAUD)
        print(f'转盘已连接 {TURNTABLE_PORT}，设置工具1零点。')
        tt.set_zero()
        tt.enable()
        if not tt.enabled or tt.hold_fault:
            raise RuntimeError(tt.hold_fault or '转盘使能未确认')
        if not wf.run_full(controller.ser, tt):
            controller.emergency_stop()
            print('D1–D3 未完成，停止本轮。')
            return False
        print(f'本轮 D1–D3 完成，已回到 SAFE={safe}；不自动取下一号电极。')
        return True
    except Exception as exc:
        controller.emergency_stop()
        print(f'转盘或 D1–D3 失败，停止本轮：{exc}')
        return False
    finally:
        if tt is not None:
            tt.close()


def visual_and_drill(controller):
    if not visual_position(controller):
        return False
    return drill_after_selection(controller)


def run_startup(controller):
    if not home(controller):
        print('归零或坐标检查失败，流程停止。')
        return False
    port_number = choose_feed_port()
    if port_number is None:
        print('已取消取料，未开始视觉流程。')
        return False
    print(f'本轮取 {port_number} 号供料口，定位速度 {POSITIONING_SPEED} 步/秒。')
    if not pickup_electrode(controller, port_number):
        print('取料或抬升失败，未开始视觉流程。')
        return False
    if not confirm('是否开始视觉识别流程？'):
        print('已停在取料后的抬升位置。可输入 VIS 开始视觉流程。')
        return True
    return visual_and_drill(controller)


def run_commands(controller):
    print('\n命令：VIS / VIS_HEAD=头部识别并选择一个点；POS=坐标；Q=退出。')
    print('头部选点到位后自动抬升至 SAFE，再询问是否开始 D1–D3。')
    while True:
        command = input('auto> ').strip().upper()
        if command in ('Q', 'QUIT', 'EXIT'):
            return
        if command in ('VIS', 'VISION', 'VIS_HEAD'):
            if not visual_and_drill(controller):
                print('本轮流程未完成；停止当前运行，请排查后重新启动。')
                return
        elif command in ('POS', 'P'):
            print(motion.query_positions(controller.ser))
        elif command in ('E', 'STOP'):
            controller.emergency_stop()
            return
        elif command:
            print('请输入 VIS、VIS_HEAD、POS 或 Q。')


def start_force_plot():
    """独立观察窗口；仅读取 CSV，启动失败不影响控制流程。"""
    root = Path(__file__).resolve().parents[2]
    try:
        import importlib.util
        if importlib.util.find_spec('matplotlib') is None or importlib.util.find_spec('tkinter') is None:
            print('力曲线窗口不可用：请安装 requirements.txt 依赖，并确认 Python 包含 Tk。')
            return None
        return subprocess.Popen(
            [sys.executable, str(root / 'tools/drill_plot/force_plot.py'),
             '--watch', str(root / 'runtime/motion_logs'), '--show-now'],
            cwd=str(root), creationflags=getattr(subprocess, 'CREATE_NO_WINDOW', 0),
        )
    except Exception as exc:
        print(f'力曲线窗口启动失败：{exc}')
        return None


def stop_force_plot(process):
    if process is None or process.poll() is not None:
        return
    try:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
    except Exception as exc:
        print(f'力曲线窗口退出失败：{exc}')


def main(argv=None):
    parser = argparse.ArgumentParser(description='选择供料口取电极与视觉定位流程')
    parser.add_argument('--serial-port', help='Arduino 串口，默认自动识别')
    args = parser.parse_args(argv)
    controller = None
    plot_process = None
    try:
        controller = MotionController(args.serial_port or motion.pick_port())
        controller.connect()
        plot_process = start_force_plot()
        print('连接成功，开始自动归零。')
        if not run_startup(controller):
            return 1
        run_commands(controller)
        return 0
    except (EOFError, KeyboardInterrupt):
        print('\n操作结束，正在停止。')
        return 130
    except Exception as exc:
        print(f'流程停止：{exc}')
        return 1
    finally:
        motion.vision_reference_ready = False
        if controller is not None:
            try:
                controller.emergency_stop()
            finally:
                try:
                    controller.close()
                finally:
                    stop_force_plot(plot_process)
        else:
            stop_force_plot(plot_process)


if __name__ == '__main__':
    raise SystemExit(main())
