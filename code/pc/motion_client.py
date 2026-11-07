# -*- coding: utf-8 -*-
"""
蟑螂自动化装配系统 —— 运动上位机 (V3)
两条控制链并行:
  1. 串口  -> Arduino Mega2560 -> 步进电机 X/Y/Z/B
  2. USB-CAN -> 转盘电机 DM-J4310-2EC (MIT 位置模式)
依赖: pip install pyserial numpy opencv-python pygrabber comtypes
用法: python motion_client.py
"""
import sys
import time
import math
import threading
import re
import json
import subprocess
from pathlib import Path
from datetime import datetime
"""20261011新增头部视觉识别"""
from vis_head import capture_and_detect_head

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_PATH = PROJECT_ROOT / 'motion_logs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.log')
motion_fault = None
vision_reference_ready = False
vis_head_fast_ready = False


def log_io(direction, message):
    LOG_PATH.parent.mkdir(parents=True, exist_ok=True)
    with LOG_PATH.open('a', encoding='utf-8') as stream:
        stream.write(f'{datetime.now().isoformat(timespec="milliseconds")} {direction} {message!r}\n')


def note_response(message):
    global motion_fault
    log_io('RX', message)
    if 'LIMIT_STOP' in message or 'HOME FAIL' in message:
        motion_fault = message

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial, 请先运行: pip install pyserial")
    sys.exit(1)

BAUD = 115200
EXPECTED_FIRMWARE_ID = 'FW motion-bootpos-p2-20260930'
# 当前 Arduino Mega 使用 CH340 USB 串口。保留官方 Arduino VID，方便以后更换板子。
ARDUINO_USB_IDS = {
    (0x1A86, 0x7523),  # CH340
    (0x2341, 0x0010),  # Arduino Mega 2560
    (0x2341, 0x0042),
    (0x2A03, 0x0010),
    (0x2A03, 0x0042),
}
# 点动每按一下走的步数 (想细一点就调小, 想快一点就调大)
# 各轴步长分开设: X/Y/Z 是丝杆(直线), B 是旋转轴(一圈 400 步),
# 用一样的步数 B 会转太多, 所以单独调小
JOG_STEP = {
    'X': 50,
    'Y': 50,
    'Z': 50,
    'B': 5,      # B: 5 步 ≈ 4.5°, 想要更细就调成 2
}

# 视觉识别固定机位（机械限位 HOME 后的绝对步数坐标）。
# 标定日期: 2026-09-30；由 BOOTPOS 首次回零实测上电位置得到。
# 参考图: vision_output/lighting_adjusted_20260930/raw_00.jpg
# 运动顺序固定为先连续移动 X/Y、再连续下降 Z，避免下降过程中横移。
VISION_POS_STEPS = {
    'X': 1666,
    'Y': -1603,
    'Z': 8908,
}
VISION_CAMERA_NAME = 'LRCP imx317'
VISION_FRAME_COUNT = 7
VISION_SAMPLE_INTERVAL_SECONDS = 2.0
VISION_CAPTURE_TIMEOUT_SECONDS = 30.0
VISION_SETTLE_SECONDS = 2.0
VISION_MIN_CONFIDENCE = 0.55
VISION_MIN_STABLE_FRAMES = 5
# 大创六点策略下，腹部点从视觉位的 X 行程实测约 1003 步；
# 单轴单次位移阈值提高到 1200 步，留出识别波动余量。
VISION_MAX_XY_MOVE_STEPS = 1200
VISION_HEAD_MAX_XY_MOVE_STEPS = 1000
VISION_PIXEL_CALIBRATION = PROJECT_ROOT / 'pc' / 'xy_pixel_calibration.json'
VISION_TOOL_CALIBRATION = PROJECT_ROOT / 'xy_calibration' / 'manual_left_p2_20260909.json'
VISION_POINT_COMPENSATION = PROJECT_ROOT / 'pc' / 'dachuang_point_compensation.json'
VISION_HEAD_POINT_COMPENSATION = PROJECT_ROOT / 'pc' / 'dachuang_head_point_compensation.json'
VISION_OUTPUT_ROOT = PROJECT_ROOT / 'vision_output' / 'auto'
VISION_HEAD_OUTPUT_ROOT = PROJECT_ROOT / 'vision_output' / 'head_auto'
VISION_SHOW_RESULT = True

# ---------------- 转盘电机（USB-CAN）配置 ----------------
TURNTABLE_PORT = 'COM10'   # USB-CAN 模块在设备管理器里的 COM 号
TURNTABLE_BAUD = 921600
TURNTABLE_KP = 20          # 位置刚度
TURNTABLE_KD = 1.0         # 速度阻尼
TURNTABLE_SPEED = 0.6      # 转盘角速度 (rad/s)，觉得慢就调大
TURNTABLE_STARTUP_ENABLED = True  # 当前阶段先完成自动视觉XY，暂不自动连接/换头


class Turntable:
    """转盘电机控制（DM-J4310-2EC, USB-CAN, MIT 位置模式）

    注意：MIT 模式下电机不主动保持位置，必须持续发 controlMIT 帧才能
    "锁住"位置。这里开一个后台线程以 100Hz 持续下发当前目标角度，
    这样使能后电机会锁死（手转不动），转到某工具后也一直停在原位。
    """
    TOOL_RAD = [0.0, math.radians(120), math.radians(240)]   # 工具1/2/3 的绝对角度

    def __init__(self, port, baud):
        from DM_CAN import Motor, MotorControl, DM_Motor_Type
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.motor = Motor(DM_Motor_Type.DM4310, 0x01, 0x11)
        self.ctrl = MotorControl(self.ser)
        self.ctrl.addMotor(self.motor)
        self.angle = 0.0      # 当前绝对角度(rad)，连续值
        self.tool = 1         # 当前工具号 1/2/3
        self.enabled = False
        self.zeroed = False   # 是否已设过零点(TZ)
        self._target = 0.0    # 目标角度，后台线程持续下发这个值
        self._stop = threading.Event()
        self._thread = None
        self.hold_fault = None

    # ---- 后台保持线程：持续发 controlMIT 锁住位置 ----
    def _hold_loop(self):
        try:
            while not self._stop.is_set():
                self.ctrl.controlMIT(self.motor, TURNTABLE_KP, TURNTABLE_KD, self._target, 0, 0)
                stamp = self.motor.feedback_time
                if stamp is None or time.monotonic() - stamp > 1.0:
                    raise RuntimeError('转盘反馈超时')
                self._stop.wait(0.01)   # 100Hz
        except Exception as exc:
            self.hold_fault = str(exc)
            self.enabled = False
            self._stop.set()
            print('转盘保持故障，禁止继续下降:', exc)

    def _start_hold(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._hold_loop, daemon=True)
            self._thread.start()

    def _stop_hold(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    def read_actual_angle(self, timeout=2.0):
        if self._thread is not None and self._thread.is_alive():
            raise RuntimeError('保持线程运行时不能并发查询串口')
        self.ser.reset_input_buffer()
        self.ctrl.data_save = b''
        started = time.monotonic()
        while time.monotonic() - started < timeout:
            self.ctrl.refresh_motor_status(self.motor)
            time.sleep(0.05)
            self.ctrl.recv()
            stamp = self.motor.feedback_time
            if stamp is not None and stamp >= started:
                angle = self.motor.getPosition()
                velocity = self.motor.getVelocity()
                if not math.isfinite(angle) or not -12.5 < angle < 12.5:
                    raise RuntimeError('转盘角度反馈无效')
                if not math.isfinite(velocity) or abs(velocity) > 0.1:
                    raise RuntimeError('转盘尚未静止')
                return angle
        raise RuntimeError('没有收到新鲜转盘反馈，拒绝按默认零角度使能')

    def enable(self):
        if self.enabled:
            return
        self._stop_hold()
        self.hold_fault = None
        angle = self.read_actual_angle()
        self.angle = self._target = angle
        try:
            # 先清除可能残留的运动设定，再使能并锁住实测角度。
            self.ctrl.controlMIT(self.motor, 0, 0, angle, 0, 0)
            self.ctrl.enable(self.motor)
            self.ctrl.controlMIT(self.motor, TURNTABLE_KP, TURNTABLE_KD, angle, 0, 0)
            self.enabled = True
            self._start_hold()
            time.sleep(0.3)
            if not self.enabled or self.hold_fault:
                raise RuntimeError(self.hold_fault or '转盘保持失败')
        except Exception:
            self.disable()
            raise
        print(f'转盘已使能，保持实测角度 {math.degrees(angle):.2f}°；工具编号需独立标定')

    def disable(self):
        self._stop_hold()
        self.ctrl.disable(self.motor)
        self.enabled = False
        print("转盘已失能（红灯）")

    def set_zero(self):
        if self.enabled:                # 设零点应在失能态，先停
            self.disable()
        self.ctrl.set_zero_position(self.motor)
        self.angle = 0.0
        self.tool = 1
        self._target = 0.0
        self.zeroed = True
        print("已设零点：当前位置 = 工具1 = 0°（请再 TE 使能）")

    def goto_tool(self, n):
        if not self.zeroed:
            print('工具零点尚未建立，禁止自动换头；当前角度保持不受影响')
            return
        if not self.enabled:
            print("转盘未使能，请先输入 TE")
            return
        n = int(n)
        if n not in (1, 2, 3):
            print("工具号只能是 1 / 2 / 3")
            return
        target0 = self.TOOL_RAD[n - 1]
        # 选离当前角度最近的目标表示，保证每次转 <= 180°（工具间总是 ±120°）
        k = round((self.angle - target0) / (2 * math.pi))
        target = target0 + k * 2 * math.pi
        delta = target - self.angle
        print(f"转盘 -> 工具{n}（转 {math.degrees(delta):+.0f}°）")
        self._ramp_target(self.angle, target)
        self.angle = target
        self.tool = n
        time.sleep(0.3)

    def next_tool(self):
        self.goto_tool(self.tool % 3 + 1)          # 1 -> 2 -> 3 -> 1

    def prev_tool(self):
        self.goto_tool((self.tool - 2) % 3 + 1)    # 1 -> 3 -> 2 -> 1

    def _ramp_target(self, start, end):
        """平滑更新目标角度（后台线程负责实际发送），角速度恒定"""
        delta = end - start
        duration = abs(delta) / TURNTABLE_SPEED
        steps = max(int(duration / 0.01), 1)
        for i in range(steps + 1):
            self._target = start + delta * i / steps
            time.sleep(0.01)
        self._target = end

    def close(self):
        try:
            self.disable()
            self.ser.close()
        except Exception:
            pass


def handle_turntable(tt, line):
    """处理转盘指令，返回 True 表示已识别"""
    up = line.upper()
    if up == 'TE':
        tt.enable()
    elif up == 'TD':
        tt.disable()
    elif up == 'TZ':
        tt.set_zero()
    elif up == 'TN':
        tt.next_tool()
    elif up == 'TP':
        tt.prev_tool()
    elif up in ('T1', 'T2', 'T3'):
        tt.goto_tool(int(up[1]))
    else:
        return False
    return True


def pick_port():
    all_ports = list(list_ports.comports())
    if not all_ports:
        print("没有检测到串口, 请确认 Arduino 已插上 USB")
        sys.exit(1)
    # 排除蓝牙端口和转盘(USB-CAN)端口，避免误选
    ports = []
    for p in all_ports:
        desc = (p.description or '').lower()
        if p.device.upper() == TURNTABLE_PORT.upper():
            continue                      # 转盘口
        if '蓝牙' in desc or 'bluetooth' in desc:
            continue                      # 蓝牙口
        ports.append(p)
    if not ports:
        ports = all_ports                 # 都过滤完了就全显示

    # 这台设备同时接有 FTDI/RS485 等串口。只有一个明显的 Arduino/CH340 时
    # 直接选它，避免用户误选后“能写入但永远没有 Arduino 回包”。
    likely_arduino = [
        p for p in ports
        if (p.vid, p.pid) in ARDUINO_USB_IDS
        or 'arduino' in (p.description or '').lower()
        or 'ch340' in (p.description or '').lower()
    ]
    if len(likely_arduino) == 1:
        selected = likely_arduino[0]
        print(f"自动识别 Arduino: {selected.device} - {selected.description}")
        return selected.device

    print("检测到串口(已排除蓝牙和转盘口):")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    while True:
        sel = input(f"选择 Arduino 串口序号 [0-{len(ports)-1}]: ").strip()
        if sel.isdigit() and 0 <= int(sel) < len(ports):
            return ports[int(sel)].device
        print("输入无效")


def open_verified_arduino(port):
    """连接并验证 Arduino 固件；启动时先停机，绝不静默连接其他串口设备。"""
    try:
        ser = serial.Serial(port, BAUD, timeout=1)
    except serial.SerialException as exc:
        raise RuntimeError(
            f'无法打开 {port}；请先关闭 Arduino IDE 串口监视器或其他占用该端口的程序'
        ) from exc
    time.sleep(2)              # 打开串口会让 Mega2560 复位
    startup = ser.read(ser.in_waiting).decode('utf-8', 'replace')
    if startup:
        sys.stdout.write(startup)

    if 'READY' not in startup:
        send(ser, 'DIAG')
        probe = read_response(ser, timeout=0.8)
        if probe:
            print(probe, end='')
        if 'HOMING=' not in probe or 'TARGET X=' not in probe:
            ser.close()
            raise RuntimeError(
                f'{port} 没有返回 Arduino 运动固件握手；请关闭串口监视器并确认选择 CH340 端口'
            )

    if EXPECTED_FIRMWARE_ID not in startup:
        ser.close()
        raise RuntimeError(
            f'{port} 上不是本次 ZDOWN/PWM 固件（缺少 {EXPECTED_FIRMWARE_ID}）'
        )

    # Arduino 可能一直由 USB 供电，连接时强制清掉串口监视器留下的运动/PWM状态。
    send(ser, 'E')
    stopped = read_response(ser, markers=('STOP E',), timeout=1.0)
    if stopped:
        print(stopped, end='')
    if 'STOP E' not in stopped:
        ser.close()
        raise RuntimeError(f'{port} 未确认安全停止，拒绝进入控制界面')
    return ser


def send(ser, line):
    log_io('TX', line)
    ser.write((line + "\n").encode('utf-8'))
    ser.flush()


def normalize_motion_command(line):
    """把上位机的简写转成固件协议，并在发送前校验钻头占空比。"""
    up = line.strip().upper()
    tokens = up.split()
    duty_text = None
    if len(tokens) == 2 and tokens[0] == 'PWM':
        duty_text = tokens[1]
    elif len(tokens) == 3 and tokens[:2] == ['DRILL', 'PWM']:
        duty_text = tokens[2]
    elif (len(tokens) == 2 and tokens[0] == 'DRILL'
          and tokens[1] not in ('ON', 'OFF', 'STATUS')):
        duty_text = tokens[1]

    if duty_text is not None:
        try:
            duty = int(duty_text)
        except ValueError as exc:
            raise ValueError('钻头占空比必须是 0 到 100 的整数') from exc
        if not 0 <= duty <= 100:
            raise ValueError('钻头占空比必须在 0 到 100 之间')
        return f'DRILL PWM {duty}'
    return line.strip()


def read_response(ser, markers=None, timeout=2.0):
    """读串口; 给 markers 就等到出现任一标记, 否则读一小段时间"""
    out = ""
    t0 = time.time()
    while time.time() - t0 < timeout:
        n = ser.in_waiting
        if n:
            out += ser.read(n).decode('utf-8', 'replace')
            note_response(out)
            if markers and any(m in out for m in markers):
                break
            t0 = time.time()   # 有数据就刷新等待
        else:
            time.sleep(0.01)
    return out


POS_RE = re.compile(
    r"POS X1=(-?\d+) X2=(-?\d+) Y=(-?\d+) Z=(-?\d+) B=(-?\d+)"
)


def query_positions(ser, timeout=1.0):
    """查询实时步数坐标；返回 X/X2/Y/Z/B 字典，超时返回 None。"""
    send(ser, "POS")
    out = ""
    deadline = time.time() + timeout
    while time.time() < deadline:
        n = ser.in_waiting
        if n:
            out += ser.read(n).decode('utf-8', 'replace')
            note_response(out)
            match = POS_RE.search(out)
            if match:
                x1, x2, y, z, b = map(int, match.groups())
                return {'X': x1, 'X2': x2, 'Y': y, 'Z': z, 'B': b}
        else:
            time.sleep(0.01)
    return None


def wait_positions(ser, targets, timeout=75.0):
    """等待一个或多个轴到达目标位置。"""
    deadline = time.time() + timeout
    while time.time() < deadline:
        pos = query_positions(ser)
        if motion_fault:
            print('运动故障:', motion_fault)
            return False
        if pos is not None:
            reached = all(pos[axis] == target for axis, target in targets.items())
            if 'X' in targets:
                reached = reached and pos['X2'] == targets['X']
            if reached:
                return True
        time.sleep(0.05)
    return False


def move_continuous(ser, targets):
    """使用一次 G0 连续移动到给定的绝对步数坐标。"""
    global motion_fault
    motion_fault = None
    cmd = "G0 " + " ".join(f"{axis}{target}" for axis, target in targets.items())
    send(ser, cmd)
    out = read_response(ser, timeout=0.3)
    if out:
        print(out, end="")
    if wait_positions(ser, targets):
        return True
    send(ser, "DIAG")
    print(read_response(ser, timeout=0.2), end='')
    send(ser, "STOP")
    print(read_response(ser, timeout=0.2), end='')
    print(f"\n未能到达 {targets}，已停止 VISION")
    return False


def goto_vision_position(ser):
    """HOME 后移动到固定视觉识别机位并停下；不采图、不识别、不移动目标。"""
    global vision_reference_ready
    vision_reference_ready = False
    print("VISION: 开始归零（Z -> Y -> X）")
    send(ser, "HOME")
    out = read_response(ser, markers=("HOME DONE", "HOME FAIL"), timeout=90)
    print(out, end="")
    if "HOME DONE" not in out or "HOME FAIL" in out:
        print("VISION: 归零失败，未执行后续移动")
        return False
    vision_reference_ready = True

    print("VISION: 连续移动 X/Y，然后连续下降 Z")
    xy_target = {axis: VISION_POS_STEPS[axis] for axis in ('X', 'Y')}
    if not move_continuous(ser, xy_target):
        vision_reference_ready = False
        return False
    if not move_continuous(ser, {'Z': VISION_POS_STEPS['Z']}):
        vision_reference_ready = False
        return False
    pos = query_positions(ser)
    if pos is None or any(
        pos[axis] != VISION_POS_STEPS[axis] for axis in ('X', 'Y', 'Z')
    ):
        vision_reference_ready = False
        print(f'VISION: 未能确认已到达视觉位：{pos}')
        return False
    print(f"VISION READY: {pos}；已停在视觉位，等待下一条命令")
    return True


def goto_vision_position_without_home(ser):
    """在本次运行已确认机械零点时，跳过 HOME 并安全返回视觉位。"""
    global vision_reference_ready
    if not vision_reference_ready:
        print('VIS_FAST: 本次程序运行尚未确认机械零点，请先执行一次 VIS 或 HOME')
        return False

    pos = query_positions(ser)
    if pos is None:
        print('VIS_FAST: 无法读取当前 POS，拒绝移动')
        return False
    if motion_fault:
        print(f'VIS_FAST: 当前存在运动故障，拒绝移动：{motion_fault}')
        return False
    if pos['X'] != pos['X2']:
        print(f'VIS_FAST: X1/X2 位置不一致，拒绝移动：{pos}')
        return False

    expected = VISION_POS_STEPS
    if all(pos[axis] == expected[axis] for axis in ('X', 'Y', 'Z')):
        print(f'VIS_FAST READY: {pos}；当前已在视觉位，直接开始识别')
        return True

    print('VIS_FAST: 跳过归零，先提升 Z，再移动 X/Y，最后下降到视觉高度')
    if pos['Z'] != 20 and not move_continuous(ser, {'Z': 20}):
        vision_reference_ready = False
        return False
    xy_target = {axis: expected[axis] for axis in ('X', 'Y')}
    if pos['X'] != expected['X'] or pos['Y'] != expected['Y']:
        if not move_continuous(ser, xy_target):
            vision_reference_ready = False
            return False
    if not move_continuous(ser, {'Z': expected['Z']}):
        vision_reference_ready = False
        return False

    final_pos = query_positions(ser)
    if final_pos is None or any(
        final_pos[axis] != expected[axis] for axis in ('X', 'Y', 'Z')
    ):
        vision_reference_ready = False
        print(f'VIS_FAST: 未能到达视觉位，拒绝识别：{final_pos}')
        return False
    print(f'VIS_FAST READY: {final_pos}；已跳过归零并到达视觉位')
    return True


def calculate_vision_target(
    payload,
    calibration,
    tool_calibration,
    point_index=None,
    point_compensation=None,
):
    """把用户选择的识别点换算为带工具偏移的绝对 XY 目标。"""
    vision_dir = PROJECT_ROOT / '视觉'
    if str(vision_dir) not in sys.path:
        sys.path.insert(0, str(vision_dir))
    from xy_coordinate_transform import pixel_to_absolute_xy

    if point_index is None:
        point_index = int(tool_calibration['selected_result_point_index'])
    else:
        point_index = int(point_index)
    points = payload.get('points', [])
    if point_index < 0 or point_index >= len(points):
        raise ValueError(f'视觉结果没有目标点索引 {point_index}')
    point = points[point_index]
    capture_xy = (VISION_POS_STEPS['X'], VISION_POS_STEPS['Y'])
    camera_alignment = pixel_to_absolute_xy(
        tuple(point), capture_xy, calibration=calibration
    )
    tool_offset = tool_calibration['tool_offset_from_reference_steps']
    target_before_point_compensation = [
        camera_alignment['absolute_steps_float'][axis] + float(tool_offset[axis])
        for axis in (0, 1)
    ]
    correction = {'X': 0, 'Y': 0}
    if point_compensation is not None:
        configured = point_compensation.get(
            'correction_steps_by_point_number', {}
        ).get(str(point_index + 1), {})
        correction = {
            'X': int(configured.get('X', 0)),
            'Y': int(configured.get('Y', 0)),
        }
    target_float = [
        target_before_point_compensation[0] + correction['X'],
        target_before_point_compensation[1] + correction['Y'],
    ]
    target = [int(round(value)) for value in target_float]
    relative = [target[axis] - capture_xy[axis] for axis in (0, 1)]
    print(
        f'VISION: 公共针头偏移 X={float(tool_offset[0]):+.2f} '
        f'Y={float(tool_offset[1]):+.2f}；'
        f'点位 {point_index + 1} 补偿 X={correction["X"]:+d} '
        f'Y={correction["Y"]:+d} 步'
    )
    return {
        'selected_result_point_index': point_index,
        'selected_point_px': point,
        'camera_alignment': camera_alignment,
        'tool_offset_steps': tool_offset,
        'target_before_point_compensation_steps': {
            'X': int(round(target_before_point_compensation[0])),
            'Y': int(round(target_before_point_compensation[1])),
        },
        'point_compensation_steps': correction,
        'target_xy_steps_float': target_float,
        'target_xy_steps': {'X': target[0], 'Y': target[1]},
        'relative_from_vision_steps': {'X': relative[0], 'Y': relative[1]},
    }


def check_vision_xy_move(ser, target, max_xy_move_steps=VISION_MAX_XY_MOVE_STEPS):
    """Check each XY displacement against a fresh machine position."""
    current = query_positions(ser)
    if current is None:
        raise ValueError('无法读取当前 POS，拒绝移动')
    if motion_fault:
        raise ValueError(f'运动故障，拒绝移动：{motion_fault}')
    if current['X'] != current['X2']:
        raise ValueError(f'X1/X2 位置不一致，拒绝移动：{current}')
    if current['Z'] != VISION_POS_STEPS['Z']:
        raise ValueError(f'Z 已离开视觉高度，拒绝移动：{current}')
    delta = {axis: target[axis] - current[axis] for axis in ('X', 'Y')}
    if any(abs(value) > max_xy_move_steps for value in delta.values()):
        raise ValueError(
            f'本次位移 {delta} 超过安全阈值 ±{max_xy_move_steps} 步；'
            f'当前 XY=({current["X"]}, {current["Y"]})，目标 XY={target}'
        )
    print(f'VISION: 当前 XY=({current["X"]}, {current["Y"]})，本次位移={delta}')
    return current, delta


def prompt_vision_point_index(points, allowed_indices, prompt):
    """Prompt for one 1-based point number and return its zero-based index."""
    allowed_indices = sorted(allowed_indices)
    while True:
        for index in allowed_indices:
            print(f'  点位 {index + 1}: 像素坐标 {points[index]}')
        choice = input(prompt).strip().lower()
        if choice in ('q', 'quit', 'exit'):
            return None
        if choice.isdigit():
            index = int(choice) - 1
            if index in allowed_indices:
                return index
        print('输入无效，请输入列表中的点位编号，或输入 Q 结束。')


def capture_and_calculate_vision_target():
    """采集7帧、识别并保存结果，常驻显示结果并返回绝对XY目标。"""
    vision_dir = PROJECT_ROOT / '视觉'
    if str(vision_dir) not in sys.path:
        sys.path.insert(0, str(vision_dir))
    from abdomen_spot_detector import imwrite_unicode
    from dachuang_strategy import detect_multiframe, build_capture_preview
    from live_abdomen_detection import capture_frames

    print(f'VISION: 机构稳定 {VISION_SETTLE_SECONDS:.0f}s 后开始约20s视觉识别')
    time.sleep(VISION_SETTLE_SECONDS)
    frames = capture_frames(
        camera_name=VISION_CAMERA_NAME,
        count=VISION_FRAME_COUNT,
        interval_seconds=VISION_SAMPLE_INTERVAL_SECONDS,
        timeout_seconds=VISION_CAPTURE_TIMEOUT_SECONDS,
        show_preview=VISION_SHOW_RESULT,
        preview_width=1280,
        preview_builder=build_capture_preview,
    )
    result = detect_multiframe(frames)
    successful = int(result['successful_frames'])
    stable = int(result['stable_frames'])
    confidence = float(result['confidence'])
    if successful != VISION_FRAME_COUNT:
        raise ValueError(f'仅 {successful}/{VISION_FRAME_COUNT} 帧识别成功，拒绝移动')
    if stable < VISION_MIN_STABLE_FRAMES:
        raise ValueError(f'仅 {stable}/{successful} 帧稳定，拒绝移动')
    if confidence < VISION_MIN_CONFIDENCE:
        raise ValueError(
            f'置信度 {confidence:.3f} 低于 {VISION_MIN_CONFIDENCE:.2f}，拒绝移动'
        )

    calibration = json.loads(VISION_PIXEL_CALIBRATION.read_text(encoding='utf-8'))
    tool_calibration = json.loads(VISION_TOOL_CALIBRATION.read_text(encoding='utf-8'))
    point_compensation = json.loads(
        VISION_POINT_COMPENSATION.read_text(encoding='utf-8')
    )
    points = result.get('points', [])
    if not points:
        raise ValueError('视觉结果中没有可供选择的目标点')
    print(f'VISION: 本次识别到 {len(points)} 个候选点')
    point_index = prompt_vision_point_index(
        points,
        range(len(points)),
        '请选择第一个要移动到的点位编号（输入 Q 取消）：',
    )
    if point_index is None:
        raise ValueError('用户取消点位选择')
    calculation = calculate_vision_target(
        result,
        calibration,
        tool_calibration,
        point_index=point_index,
        point_compensation=point_compensation,
    )

    run_dir = VISION_OUTPUT_ROOT / datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir.mkdir(parents=True, exist_ok=True)
    for index, (raw, frame_result) in enumerate(zip(frames, result['frame_results'])):
        imwrite_unicode(run_dir / f'raw_{index:02d}.jpg', raw)
        imwrite_unicode(run_dir / f'annotated_{index:02d}.jpg', frame_result.annotated)
    payload = {key: value for key, value in result.items() if key != 'frame_results'}
    record = {
        'status': 'validated_ready_to_move',
        'capture_machine_steps': VISION_POS_STEPS,
        'detection': payload,
        'calculation': calculation,
        'point_movements': [],
        'selected_point_order': [point_index + 1],
        'pixel_calibration': str(VISION_PIXEL_CALIBRATION),
        'tool_calibration': str(VISION_TOOL_CALIBRATION),
        'point_compensation': str(VISION_POINT_COMPENSATION),
    }
    (run_dir / 'result.json').write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print(
        f"VISION: {successful}/{VISION_FRAME_COUNT}帧成功，{stable}帧稳定，"
        f"置信度={confidence:.3f}"
    )
    print(f"VISION: 目标点={calculation['selected_point_px']}")
    print(f"VISION: 计算记录={run_dir.resolve()}")
    # 结果窗口使用独立进程显示，避免阻塞后续自动XY移动；窗口会一直保留，
    # 直到用户点关闭按钮或按 Q / Esc。该预览只读取本轮已保存图像，不占相机。
    preview_script = vision_dir / 'live_abdomen_detection.py'
    preview_args = [
        sys.executable,
        str(preview_script),
        '--result-json',
        str(run_dir / 'result.json'),
        '--preview-width',
        '1280',
    ]
    creationflags = 0
    if sys.platform == 'win32':
        creationflags = subprocess.CREATE_NEW_PROCESS_GROUP
    subprocess.Popen(
        preview_args,
        cwd=str(PROJECT_ROOT),
        creationflags=creationflags,
    )
    print('VISION: 结果窗口已常驻显示；关闭窗口或按 Q / Esc 可退出预览')
    return {
        'first_calculation': calculation,
        'payload': payload,
        'calibration': calibration,
        'tool_calibration': tool_calibration,
        'point_compensation': point_compensation,
        'record': record,
        'result_path': run_dir / 'result.json',
        'remaining_indices': set(range(len(points))) - {point_index},
    }


def run_automatic_vision(ser, skip_home=False):
    """进入视觉位、稳定识别并移动到针轴 XY；不换头、不动目标 Z。"""
    position_ready = (
        goto_vision_position_without_home(ser)
        if skip_home else goto_vision_position(ser)
    )
    if not position_ready:
        return False
    try:
        session = capture_and_calculate_vision_target()
        target = session['first_calculation']['target_xy_steps']
    except Exception as exc:
        print(f'VISION: 识别或换算失败，停在视觉位，未发送目标移动：{exc}')
        return False

    pos = query_positions(ser)
    expected = VISION_POS_STEPS
    if pos is None or any(pos[axis] != expected[axis] for axis in ('X', 'Y', 'Z')):
        print(f'VISION: 识别后机械位置异常 {pos}，拒绝发送目标移动')
        return False
    print(f"VISION: 自动移动到目标 X={target['X']} Y={target['Y']}（Z保持视觉高度）")
    try:
        move_start, move_delta = check_vision_xy_move(ser, target)
    except ValueError as exc:
        print(f'VISION: 未发送目标移动：{exc}')
        return False
    if not move_continuous(ser, target):
        return False
    final_pos = query_positions(ser)
    record = session['record']
    result_path = session['result_path']
    selected_moves = record['point_movements']
    first_index = session['first_calculation']['selected_result_point_index']
    selected_moves.append({
        'point_number': first_index + 1,
        'calculation': session['first_calculation'],
        'move_start_machine_steps': move_start,
        'relative_from_current_steps': move_delta,
        'final_machine_steps': final_pos,
    })
    record['status'] = 'point_reached_waiting_for_next_selection'
    result_path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print(f'VISION: 点位 {first_index + 1} 已到达，当前位置={final_pos}')

    remaining = session['remaining_indices']
    while remaining:
        next_index = prompt_vision_point_index(
            session['payload']['points'],
            remaining,
            '请选择下一个要移动到的点位编号（输入 Q 结束）：',
        )
        if next_index is None:
            break
        try:
            calculation = calculate_vision_target(
                session['payload'],
                session['calibration'],
                session['tool_calibration'],
                point_index=next_index,
                point_compensation=session['point_compensation'],
            )
            move_start, move_delta = check_vision_xy_move(
                ser, calculation['target_xy_steps']
            )
        except Exception as exc:
            print(f'点位 {next_index + 1} 换算失败，未移动：{exc}')
            continue

        target = calculation['target_xy_steps']
        print(
            f"VISION: 移动到点位 {next_index + 1}，"
            f"X={target['X']} Y={target['Y']}（Z保持视觉高度）"
        )
        if not move_continuous(ser, target):
            record['status'] = 'movement_failed'
            result_path.write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
            )
            return False

        final_pos = query_positions(ser)
        selected_moves.append({
            'point_number': next_index + 1,
            'calculation': calculation,
            'move_start_machine_steps': move_start,
            'relative_from_current_steps': move_delta,
            'final_machine_steps': final_pos,
        })
        remaining.remove(next_index)
        record['calculation'] = calculation
        record['selected_point_order'].append(next_index + 1)
        record['status'] = (
            'all_detected_points_visited' if not remaining
            else 'point_reached_waiting_for_next_selection'
        )
        result_path.write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
        )
        print(f'VISION: 点位 {next_index + 1} 已到达，当前位置={final_pos}')

    record['status'] = 'selected_points_completed'
    result_path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print(f'VISION: 点位移动记录={result_path}')
    print(f'VISION COMPLETE: {final_pos}；本阶段未换头、未下降Z')
    return True

"""20261011新增头部视觉识别"""
def run_visual_head(ser, skip_home=False):
    """进入视觉位，识别头部四点，并按用户选择逐点移动。"""
    global vis_head_fast_ready
    if skip_home and not vis_head_fast_ready:
        print('VIS_HEAD_FAST: 本次程序运行尚未完成过 VIS_HEAD，请先执行一次 VIS_HEAD')
        return False
    position_ready = (
        goto_vision_position_without_home(ser)
        if skip_home else goto_vision_position(ser)
    )
    if not position_ready:
        return False

    try:
        points = capture_and_detect_head()
        if len(points) != 4:
            raise ValueError(f'应识别到 4 个头部点，实际得到 {len(points)} 个')
        calibration = json.loads(VISION_PIXEL_CALIBRATION.read_text(encoding='utf-8'))
        tool_calibration = json.loads(VISION_TOOL_CALIBRATION.read_text(encoding='utf-8'))
        head_point_compensation = json.loads(
            VISION_HEAD_POINT_COMPENSATION.read_text(encoding='utf-8')
        )
    except Exception as exc:
        print(f'VIS_HEAD: 识别失败，停在视觉位：{exc}')
        return False

    payload = {'points': points}
    remaining = set(range(len(points)))
    record = {
        'status': 'validated_ready_to_move',
        'capture_machine_steps': dict(VISION_POS_STEPS),
        'detected_points_px': points,
        'point_movements': [],
        'selected_point_order': [],
        'pixel_calibration': str(VISION_PIXEL_CALIBRATION),
        'tool_calibration': str(VISION_TOOL_CALIBRATION),
        'point_compensation': str(VISION_HEAD_POINT_COMPENSATION),
    }
    run_dir = VISION_HEAD_OUTPUT_ROOT / datetime.now().strftime('%Y%m%d_%H%M%S')
    run_dir.mkdir(parents=True, exist_ok=True)
    result_path = run_dir / 'result.json'
    result_path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
    )

    print('VIS_HEAD: 本次识别到 4 个可选点，使用与 VIS 相同的相机标定和针头偏移')
    final_pos = query_positions(ser)
    first_selection = True
    while remaining:
        prompt = (
            '请选择第一个要移动到的头部点编号（输入 Q 取消）：'
            if first_selection
            else '请选择下一个要移动到的头部点编号（输入 Q 结束）：'
        )
        point_index = prompt_vision_point_index(points, remaining, prompt)
        if point_index is None:
            record['status'] = (
                'selection_cancelled_before_move'
                if first_selection else 'selected_points_completed'
            )
            result_path.write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
            )
            if first_selection:
                print('VIS_HEAD: 已取消，未发送目标移动')
                return False
            break

        try:
            calculation = calculate_vision_target(
                payload,
                calibration,
                tool_calibration,
                point_index=point_index,
                point_compensation=head_point_compensation,
            )
            target = calculation['target_xy_steps']
            move_start, move_delta = check_vision_xy_move(
                ser, target, max_xy_move_steps=VISION_HEAD_MAX_XY_MOVE_STEPS
            )
        except Exception as exc:
            print(f'头部点 P{point_index + 1} 换算或检查失败，未移动：{exc}')
            continue

        print(
            f"VIS_HEAD: 移动到 P{point_index + 1}："
            f"X={target['X']} Y={target['Y']}（Z 保持视觉高度）"
        )
        if not move_continuous(ser, target):
            record['status'] = 'movement_failed'
            result_path.write_text(
                json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
            )
            return False

        final_pos = query_positions(ser)
        record['point_movements'].append({
            'point_number': point_index + 1,
            'calculation': calculation,
            'move_start_machine_steps': move_start,
            'relative_from_current_steps': move_delta,
            'final_machine_steps': final_pos,
        })
        record['selected_point_order'].append(point_index + 1)
        remaining.remove(point_index)
        first_selection = False
        record['status'] = (
            'all_detected_points_visited'
            if not remaining else 'point_reached_waiting_for_next_selection'
        )
        result_path.write_text(
            json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
        )
        print(f'VIS_HEAD: P{point_index + 1} 已到达，当前位置={final_pos}')

    record['status'] = (
        'all_detected_points_visited'
        if not remaining else 'selected_points_completed'
    )
    result_path.write_text(
        json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    print(f'VIS_HEAD: 点位移动记录={result_path}')
    print(f'VIS_HEAD COMPLETE: {final_pos}；本阶段未换头、未下降 Z')
    vis_head_fast_ready = True
    return True


def cmd_mode(ser, tt):
    global vision_reference_ready
    while True:
        try:
            line = input(">>> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("exit", "quit"):
            break
        if line.lower() == "jog":
            jog_mode(ser)
            print_usage()
            continue
        if not line:
            continue
        if line.upper() in ('VISION', 'VIS'):
            run_automatic_vision(ser)
            continue
        if line.upper() in ('VIS_FAST', 'VISF'):
            run_automatic_vision(ser, skip_home=True)
            continue
        """20261011新增头部视觉识别"""
        if line.upper() == 'VIS_HEAD':
            run_visual_head(ser)
            continue
        if line.upper() in ('VIS_HEAD_FAST', 'VIS_HEADF'):
            run_visual_head(ser, skip_home=True)
            continue
        # 转盘指令（以 T 开头，走 USB-CAN，不发给 Arduino）
        if line.upper().startswith('T'):
            if tt is None:
                print(f"转盘未连接（{TURNTABLE_PORT} 不可用），本次仅控制步进")
            elif not handle_turntable(tt, line):
                print("未知转盘指令。可用: TE / TD / TZ / T1 / T2 / T3 / TN / TP")
            continue
        try:
            command = normalize_motion_command(line)
        except ValueError as exc:
            print(f'输入错误：{exc}')
            continue
        up = command.upper()
        if up in ('HOME', 'HX', 'HY', 'HZ', 'ZERO'):
            vision_reference_ready = False
        send(ser, command)
        # 回零是阻塞式, 等各自结束标志, 输出完整不乱序
        # HOME 要等 "HOME DONE"(总结束), 不能碰到第一个 "HOMED Z" 就提前停
        home_markers = {
            "HOME": ("HOME DONE", "FAIL"),
            "HX":   ("HOMED X",   "FAIL"),
            "HY":   ("HOMED Y",   "FAIL"),
            "HZ":   ("HOMED Z",   "FAIL"),
        }
        if up in home_markers:
            out = read_response(ser, markers=home_markers[up], timeout=60)
        else:
            out = read_response(ser, timeout=0.3)
        print(out, end="")
        if up == 'HOME':
            vision_reference_ready = 'HOME DONE' in out and 'FAIL' not in out


def jog_mode(ser):
    import msvcrt
    print("\n=== 点动模式 ===")
    print("  ↑↓   = X轴 (前/后)")
    print("  ←→   = Y轴 (左/右)")
    print("  W/S  = Z轴 (上/下)    W=上  S=下")
    print("  A/D  = B轴 (旋转)     A=负  D=正")
    print("  q = 退出点动\n")
    while True:
        while ser.in_waiting:      # 清掉上一步响应
            ser.read(ser.in_waiting)

        if not msvcrt.kbhit():
            time.sleep(0.02)
            continue

        ch = msvcrt.getwch()
        if ch in ('\xe0', '\x00'):        # 方向键是两字节
            ch = msvcrt.getwch()
        else:
            ch = ch.lower()               # 普通字母统一转小写

        if ch == 'q':
            break

        cmd = key_to_cmd(ch)
        if cmd:
            send(ser, cmd)


def key_to_cmd(ch):
    m = {
        # 方向键 (扫描码, 大写)
        'H': f"X{JOG_STEP['X']}",     # ↑  X+ (前)
        'P': f"X-{JOG_STEP['X']}",    # ↓  X- (后)
        'K': f"Y{JOG_STEP['Y']}",     # ←  Y+ (左)
        'M': f"Y-{JOG_STEP['Y']}",    # →  Y- (右)
        # 字母键 (小写)
        'w': f"Z-{JOG_STEP['Z']}",    # Z 上
        's': f"Z{JOG_STEP['Z']}",     # Z 下
        'a': f"B-{JOG_STEP['B']}",    # B 负
        'd': f"B{JOG_STEP['B']}",     # B 正
    }
    return m.get(ch)


def print_usage():
    print("\n================ 使用说明 ================")
    print("【步进轴】(串口 -> Arduino)")
    print("  X100 / Y-200 / Z50 / B100   相对移动(步)")
    print("  G0 X1000 Y2000 Z500 B0    绝对坐标移动(步)")
    print("  M0 X50 Y30 Z10 B45        绝对坐标移动(mm/度)")
    print("  ALIGN                     X1/X2 计数对齐")
    print("  HOME / HX / HY / HZ       限位回零")
    print("  VISION (VIS)              归零→视觉位→约20秒可视化检测→自动移动目标XY")
    print("  VIS_FAST (VISF)           已确认零点时跳过归零，安全返回视觉位后识别")
    print("  VIS_HEAD                  归零识别头部4点，并按编号逐点移动XY")
    print("  VIS_HEAD_FAST (VIS_HEADF) 已完成VIS_HEAD时跳过归零，再次识别头部4点")
    print("  SPEED500 (V500)        设置速度")
    print("  ZDOWN (ZD)             Z轴持续向下，直到停止或触发下限位")
    print("  E                      单字母安全停止（立即停全部运动和钻头）")
    print("  POS / MPOS / ZERO / LIM / STOP / ?")
    print("  BOOTPOS (BP)             查询上电位置估算及首次回零净步数")
    print("  jog                     进入方向键点动模式")
    print("【钻孔工具头】(Arduino 25kHz PWM)")
    print("  PWM 80 / DRILL 80       设置占空比0..100（数值越大转速越高）")
    print("  DRILL ON / OFF / STATUS 启动 / 停止 / 查询当前占空比")
    print("【转盘】(USB-CAN)")
    print("  TZ  设零点(当前位置=工具1)   TE  使能   TD  失能")
    print("  T1/T2/T3  转到工具1/2/3      TN 下一工具   TP 上一工具")
    print("  建议顺序: TZ -> TE -> T1/T2/T3/TN/TP")
    print("  exit                     退出")
    print("==========================================\n")


def main():
    port = pick_port()
    try:
        ser = open_verified_arduino(port)
    except RuntimeError as exc:
        print(f'Arduino 连接失败：{exc}')
        return
    print(f"\n已连接 {port} @ {BAUD}")
    print(f'运动日志: {LOG_PATH}')

    # 转盘本阶段默认不启动；待工具位和换头安全顺序完善后再接入自动流程。
    tt = None
    if TURNTABLE_STARTUP_ENABLED:
        try:
            tt = Turntable(TURNTABLE_PORT, TURNTABLE_BAUD)
            tt.enable()
            print(f"转盘已连接 {TURNTABLE_PORT} @ {TURNTABLE_BAUD}，已保持当前位置")
        except Exception as e:
            print(f"转盘启动保持失败（{e}），继续进入仅步进/视觉模式")
            if tt is not None:
                tt.close()
            tt = None
    else:
        print("转盘自动控制本阶段未启用；VIS会识别并移动XY，但不换头、不下降Z")

    print_usage()
    try:
        cmd_mode(ser, tt)
    finally:
        # 即使用户按 Ctrl+C、输入 exit 或程序异常，也先让 Arduino 立即停机。
        try:
            send(ser, 'E')
            out = read_response(ser, timeout=0.3)
            if out:
                print(out, end='')
        except Exception as exc:
            print(f'退出时发送安全停止失败：{exc}')
        if tt is not None:
            tt.close()
        ser.close()


if __name__ == "__main__":
    main()
