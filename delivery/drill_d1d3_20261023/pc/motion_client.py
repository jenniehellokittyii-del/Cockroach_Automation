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
"""20261016新增钻孔循环：RS485 传感器阶跃 -> 停钻 -> 抬升"""
from force_sensor import ForceSensor, load_config as load_force_sensor_config
from drill_cycle import DrillParams, run_drill_cycle

PROJECT_ROOT = Path(__file__).resolve().parents[1]
LOG_PATH = PROJECT_ROOT / 'motion_logs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.log')
motion_fault = None
LAST_VISION_TARGET = None   # 本会话 VIS 成功后保存的 P2 绝对 XY（不跨会话复用）
LAST_DRILL_RESULT = None    # 本会话 DC 钻孔结果（含 z_stop / depth_steps）


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
VISION_MAX_XY_MOVE_STEPS = 800
VISION_PIXEL_CALIBRATION = PROJECT_ROOT / 'pc' / 'xy_pixel_calibration.json'
VISION_TOOL_CALIBRATION = PROJECT_ROOT / 'xy_calibration' / 'manual_left_p2_20260909.json'
VISION_OUTPUT_ROOT = PROJECT_ROOT / 'vision_output' / 'auto'
VISION_SHOW_RESULT = True

# ---------------- 钻孔循环（DC 命令）配置 ----------------
# 传感器链路参数在 pc/force_sensor.json（不存在时用 force_sensor.py 默认值）。
# 现场要改的通常是：COM 口、比例 scale、进给速度/限深/抬升距离。
FORCE_SENSOR_CONFIG_PATH = PROJECT_ROOT / 'pc' / 'force_sensor.json'
DRILL_PARAMS = DrillParams()  # 当前不转钻头（drill_duty=0），只下 Z

# ---------------- 电极接触 Z 标定 / 完整流程配置 ----------------
# 绝对安全高度（步）。Z+ 向下，数值越小越高。
# 默认 0（当前现场零点/顶部）；可被 pc/safe_z.json 覆盖。
SAFE_Z_STEPS = 0
# 转盘工具编号：TZ 时手动让"电极头"朝下，此时电极头=工具1；钻孔头=工具2（T3 为第三工具/未用）。
# 若现场对应关系不同，只改这两个数。
TOOL_ELECTRODE = 1
TOOL_DRILL = 2
# 电极下探标定：每次 100 步 = 1 mm，慢速 150 步/秒；单次最多 10000 步 = 100 mm。
ELECTRODE_PROBE_STEPS = 100
ELECTRODE_PROBE_SPEED = 150
ELECTRODE_TRAVEL_SPEED = 500   # D3 电极快速移动速度，与 D1 钻孔进给 500 步/秒相同
ELECTRODE_PROBE_MAX_STEPS = 10000
# 电极最终额外下压量（正值向下，0 = 刚好接触）。
ELECTRODE_INSERT_STEPS = 0
ELECTRODE_CALIBRATION_PATH = PROJECT_ROOT / 'pc' / 'electrode_calibration.json'

# ---------------- 转盘电机（USB-CAN）配置 ----------------
TURNTABLE_PORT = 'COM8'    # USB-CAN 模块在设备管理器里的 COM 号
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


def move_continuous(ser, targets, timeout=75.0):
    """使用一次 G0 连续移动到给定的绝对步数坐标。"""
    global motion_fault
    motion_fault = None
    cmd = "G0 " + " ".join(f"{axis}{target}" for axis, target in targets.items())
    send(ser, cmd)
    out = read_response(ser, timeout=0.3)
    if out:
        print(out, end="")
    if wait_positions(ser, targets, timeout=timeout):
        return True
    send(ser, "DIAG")
    print(read_response(ser, timeout=0.2), end='')
    send(ser, "STOP")
    print(read_response(ser, timeout=0.2), end='')
    print(f"\n未能到达 {targets}，已停止 VISION")
    return False


def goto_vision_position(ser):
    """HOME 后移动到固定视觉识别机位并停下；不采图、不识别、不移动目标。"""
    print("VISION: 开始归零（Z -> Y -> X）")
    send(ser, "HOME")
    out = read_response(ser, markers=("HOME DONE", "HOME FAIL"), timeout=90)
    print(out, end="")
    if "HOME DONE" not in out or "HOME FAIL" in out:
        print("VISION: 归零失败，未执行后续移动")
        return False

    print("VISION: 连续移动 X/Y，然后连续下降 Z")
    xy_target = {axis: VISION_POS_STEPS[axis] for axis in ('X', 'Y')}
    if not move_continuous(ser, xy_target):
        return False
    if not move_continuous(ser, {'Z': VISION_POS_STEPS['Z']}):
        return False
    pos = query_positions(ser)
    print(f"VISION READY: {pos}；已停在视觉位，等待下一条命令")
    return True


def calculate_vision_target(payload, calibration, tool_calibration):
    """把用户确认的P2点换算为带工具偏移的绝对XY目标。"""
    vision_dir = PROJECT_ROOT / '视觉'
    if str(vision_dir) not in sys.path:
        sys.path.insert(0, str(vision_dir))
    from xy_coordinate_transform import pixel_to_absolute_xy

    point_index = int(tool_calibration['selected_result_point_index'])
    points = payload.get('points', [])
    if point_index < 0 or point_index >= len(points):
        raise ValueError(f'视觉结果没有目标点索引 {point_index}')
    point = points[point_index]
    capture_xy = (VISION_POS_STEPS['X'], VISION_POS_STEPS['Y'])
    camera_alignment = pixel_to_absolute_xy(
        tuple(point), capture_xy, calibration=calibration
    )
    tool_offset = tool_calibration['tool_offset_from_reference_steps']
    target_float = [
        camera_alignment['absolute_steps_float'][axis] + float(tool_offset[axis])
        for axis in (0, 1)
    ]
    target = [int(round(value)) for value in target_float]
    relative = [target[axis] - capture_xy[axis] for axis in (0, 1)]
    if any(abs(value) > VISION_MAX_XY_MOVE_STEPS for value in relative):
        raise ValueError(f'计算位移 {relative} 超过安全阈值 ±{VISION_MAX_XY_MOVE_STEPS} 步')
    return {
        'selected_result_point_index': point_index,
        'selected_point_px': point,
        'camera_alignment': camera_alignment,
        'tool_offset_steps': tool_offset,
        'target_xy_steps_float': target_float,
        'target_xy_steps': {'X': target[0], 'Y': target[1]},
        'relative_from_vision_steps': {'X': relative[0], 'Y': relative[1]},
    }


def capture_and_calculate_vision_target():
    """采集7帧、识别并保存结果，常驻显示结果并返回绝对XY目标。"""
    vision_dir = PROJECT_ROOT / '视觉'
    if str(vision_dir) not in sys.path:
        sys.path.insert(0, str(vision_dir))
    from abdomen_spot_detector import detect_multiframe, imwrite_unicode
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
    calculation = calculate_vision_target(result, calibration, tool_calibration)

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
        'pixel_calibration': str(VISION_PIXEL_CALIBRATION),
        'tool_calibration': str(VISION_TOOL_CALIBRATION),
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
    return calculation['target_xy_steps']


def run_automatic_vision(ser):
    """HOME→视觉位→稳定识别→计算→移动到针轴XY；不换头、不动目标Z。"""
    global LAST_VISION_TARGET
    if not goto_vision_position(ser):
        return False
    try:
        target = capture_and_calculate_vision_target()
    except Exception as exc:
        print(f'VISION: 识别或换算失败，停在视觉位，未发送目标移动：{exc}')
        return False

    pos = query_positions(ser)
    expected = VISION_POS_STEPS
    if pos is None or any(pos[axis] != expected[axis] for axis in ('X', 'Y', 'Z')):
        print(f'VISION: 识别后机械位置异常 {pos}，拒绝发送目标移动')
        return False
    print(f"VISION: 自动移动到目标 X={target['X']} Y={target['Y']}（Z保持视觉高度）")
    if not move_continuous(ser, target):
        return False
    LAST_VISION_TARGET = {'X': int(target['X']), 'Y': int(target['Y'])}
    final_pos = query_positions(ser)
    print(f'VISION COMPLETE: {final_pos}；本阶段未换头、未下降Z')
    print('VISION: 本次 P2 目标已记录，可直接运行 FULL')
    return True

class SerialDrillIO:
    """把 motion_client 的串口函数包装成 drill_cycle 需要的 IO 接口。"""

    def __init__(self, ser):
        self.ser = ser

    def send(self, line):
        send(self.ser, line)

    def read_response(self, markers=None, timeout=0.3):
        return read_response(self.ser, markers=markers, timeout=timeout)

    def query_positions(self, timeout=1.0):
        return query_positions(self.ser, timeout=timeout)

    def wait_z(self, target, timeout=20.0):
        return wait_positions(self.ser, {'Z': target}, timeout=timeout)

    def fault(self):
        return motion_fault

    def clear_fault(self):
        global motion_fault
        motion_fault = None

    def log(self, message):
        print(message)
        log_io('EVT', message)


def run_automatic_drill(ser):
    """DC：传感器阶跃 → 停钻 → 抬升固定距离（不自动移动 XY）。

    前置条件：XY 已对准孔位、Z 已在待钻高度（人工或视觉流程到位）。
    传感器用 pc/force_sensor.json 配置；失败时不做任何运动。
    返回钻孔结果字典（含 z_start / z_stop / depth_steps）；传感器失败返回 None。
    """
    global LAST_DRILL_RESULT
    config = load_force_sensor_config()
    sensor = ForceSensor(config=config)
    try:
        sensor.start()
    except Exception as exc:
        print(f'DC: 传感器启动失败，未开始钻孔：{exc}')
        print('  检查：传感器 COM 口/接线/波特率；并确认 RS485 GUI 已关闭（串口独占）')
        return None
    print(f'DC: 传感器已连接（{sensor.describe()}）')
    print(f'DC: 传感器数据记录到 {sensor.csv_path}')
    try:
        io = SerialDrillIO(ser)
        result = run_drill_cycle(io, sensor, params=DRILL_PARAMS, log=io.log)
        LAST_DRILL_RESULT = result
        event = result.get('trigger')
        if isinstance(event, dict) and event.get('time') is not None:
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            window_path = PROJECT_ROOT / 'motion_logs' / f'drill_event_{stamp}.csv'
            rows = sensor.save_window(window_path, float(event['time']), before=2.0, after=2.0)
            print(f'DC: 触发前后 {rows} 个传感器样本已保存 {window_path}')
        if result['status'] == 'BREAKTHROUGH' and result['lifted']:
            print(f"DC: 完成——钻深 {result['depth_steps']} 步，已抬升 {DRILL_PARAMS.lift_steps} 步"
                  f"（Z={result['z_lift_target']}）；可用 T1/T2/T3 换头，或运行 ECAL/FULL")
        else:
            print(f"DC: 未完成——status={result['status']}，lifted={result['lifted']}，"
                  f"请人工确认机构状态后再操作")
        return result
    finally:
        sensor.close()


# ---------------- 电极接触 Z 标定 / 完整流程 ----------------

def _save_json(path, payload):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')


def _load_json(path):
    try:
        return json.loads(path.read_text(encoding='utf-8'))
    except Exception:
        return None


def _turntable_ready(tt):
    """自动换头前必须确认：已连转盘、已 TZ 零点、已 TE 使能。"""
    if tt is None:
        print('流程: 转盘未连接，无法自动换头')
        return False
    if not tt.zeroed:
        print('流程: 转盘零点未建立。请先手动把电极头转到朝下，输入 TZ，再输入 TE')
        return False
    if not tt.enabled:
        print('流程: 转盘未使能。请先输入 TE')
        return False
    return True


def _rotate_tool(tt, tool, name):
    if not _turntable_ready(tt):
        return False
    print(f'流程: 转盘转到{name}（工具{tool}）')
    tt.goto_tool(tool)
    time.sleep(0.3)
    return True


def _z_move(ser, target_z, speed=500, extra_timeout=10.0):
    """绝对移动 Z（步），按距离估算等待时间，完成后恢复全轴速度。"""
    pos = query_positions(ser)
    if pos is None:
        print('流程: 读取 POS 失败')
        return False
    distance = abs(target_z - pos['Z'])
    timeout = max(10.0, distance / max(float(speed), 1.0) * 1.5 + extra_timeout)
    send(ser, f'SPEED {int(speed)}')
    read_response(ser, timeout=0.3)
    try:
        return move_continuous(ser, {'Z': target_z}, timeout=timeout)
    finally:
        send(ser, 'SPEED 500')
        read_response(ser, timeout=0.3)


def ensure_safe_z(ser):
    """把 Z 抬到安全高度；当前已更高时保持不动，绝不往下走。"""
    pos = query_positions(ser)
    if pos is None:
        print('流程: 读取 POS 失败')
        return False
    if pos['Z'] <= SAFE_Z_STEPS:
        print(f"流程: Z={pos['Z']} 已在安全高度（≤{SAFE_Z_STEPS}）")
        return True
    print(f"流程: Z {pos['Z']} -> 安全高度 {SAFE_Z_STEPS}")
    return _z_move(ser, SAFE_Z_STEPS, speed=500)


def run_electrode_contact_test(ser, tt):
    """ECAL：转电极头慢速下探，人工确认接触，保存"接触Z - 钻停Z"偏移。"""
    if not _turntable_ready(tt):
        return False

    global LAST_DRILL_RESULT
    drill = LAST_DRILL_RESULT
    if drill is None:
        print('ECAL: 本会话还没有钻孔记录（Z_stop）。')
        text = input('  请输入本次钻孔停止时的绝对 Z（POS 里的 Z），直接回车取消: ').strip()
        if not text:
            return False
        try:
            z_stop = int(text)
        except ValueError:
            print('ECAL: Z_stop 必须是整数步数')
            return False
        drill = {'z_stop': z_stop, 'depth_steps': None, 'source': 'manual'}
    z_stop = drill.get('z_stop')
    if z_stop is None:
        print('ECAL: 钻孔记录里没有 z_stop，无法标定')
        return False

    print(f'ECAL: 使用钻停点 Z_stop={z_stop}')
    print(f'ECAL: 安全高度 Z={SAFE_Z_STEPS}，转到电极头（工具{TOOL_ELECTRODE}），'
          f'每次下探 {ELECTRODE_PROBE_STEPS} 步 = {ELECTRODE_PROBE_STEPS / 100:.2f} mm')
    if input('ECAL: 确认开始？(y/n): ').strip().lower() != 'y':
        return False

    if not ensure_safe_z(ser):
        return False
    if not _rotate_tool(tt, TOOL_ELECTRODE, '电极头'):
        return False

    send(ser, 'DRILL OFF')
    read_response(ser, timeout=0.3)
    send(ser, f'SPEED {int(ELECTRODE_PROBE_SPEED)}')
    read_response(ser, timeout=0.3)

    contact_z = None
    travelled = 0
    try:
        while travelled < ELECTRODE_PROBE_MAX_STEPS:
            pos = query_positions(ser)
            if pos is None:
                print('ECAL: 读取 POS 失败，中止')
                break
            expected = pos['Z'] + ELECTRODE_PROBE_STEPS
            send(ser, f'Z{ELECTRODE_PROBE_STEPS}')
            read_response(ser, timeout=0.3)
            if not wait_positions(ser, {'Z': expected}, timeout=15.0):
                print('ECAL: 下探未到位，已停止，请人工检查')
                send(ser, 'STOP')
                read_response(ser, timeout=0.3)
                break
            travelled += ELECTRODE_PROBE_STEPS

            pos = query_positions(ser)
            if pos is None:
                print('ECAL: 读取 POS 失败，中止')
                break
            current = pos['Z']
            relative = current - z_stop
            print(f'ECAL: 已下探 {travelled} 步，当前 Z={current}，'
                  f'相对钻停点 {relative:+d} 步（{relative / 100:+.2f} mm）')
            answer = input('  电极接触昆虫了吗？y=确认接触, 回车=继续下探, q=放弃: ').strip().lower()
            if answer == 'q':
                print('ECAL: 已放弃，不保存标定')
                break
            if answer == 'y':
                contact_z = current
                break
        else:
            print(f'ECAL: 已到上限 {ELECTRODE_PROBE_MAX_STEPS} 步仍未确认接触，中止')
    finally:
        send(ser, 'SPEED 500')
        read_response(ser, timeout=0.3)
        if not ensure_safe_z(ser):
            print('ECAL: 警告：抬升到安全高度未确认，请人工处理')

    if contact_z is None:
        return False

    offset = contact_z - z_stop
    payload = {
        'saved_at': datetime.now().isoformat(timespec='seconds'),
        'tool_electrode': TOOL_ELECTRODE,
        'tool_drill': TOOL_DRILL,
        'z_stop': z_stop,
        'z_contact': contact_z,
        'offset_steps': offset,
        'offset_mm': offset / 100.0,
        'probe_step': ELECTRODE_PROBE_STEPS,
        'insert_steps': ELECTRODE_INSERT_STEPS,
    }
    _save_json(ELECTRODE_CALIBRATION_PATH, payload)
    print(f'ECAL: 接触 Z={contact_z}，offset={offset:+d} 步（{offset / 100:+.2f} mm）')
    print(f'ECAL: 标定已保存 {ELECTRODE_CALIBRATION_PATH}')
    print('ECAL: 以后电极目标 Z = 本次钻孔 Z_stop + offset + ELECTRODE_INSERT_STEPS')
    return True


def run_full_sequence(ser, tt):
    """FULL：P2 → 钻孔(传感器) → 抬升 → 转电极头 → 下到标定接触Z → 抬升。"""
    if not _turntable_ready(tt):
        return False

    global LAST_VISION_TARGET
    target = LAST_VISION_TARGET
    if target is None:
        print('FULL: 本会话还没有视觉目标。请先运行 VIS（会归零、识别并移动到 P2）')
        return False
    calibration = _load_json(ELECTRODE_CALIBRATION_PATH)
    if not calibration or 'offset_steps' not in calibration:
        print(f'FULL: 没有电极接触标定（{ELECTRODE_CALIBRATION_PATH}），请先运行 ECAL')
        return False

    print('FULL: 将执行：')
    print(f'  1) 转到安全高度 Z={SAFE_Z_STEPS}')
    print(f"  2) XY 移动到 P2：X={target['X']} Y={target['Y']}")
    print(f'  3) 转盘转到钻孔头（工具{TOOL_DRILL}）')
    print('  4) 钻孔：DRILL ON + ZDOWN，传感器阶跃后 STOP，记录 Z_stop 并抬升')
    print(f'  5) 转盘转到电极头（工具{TOOL_ELECTRODE}）')
    print('  6) 电极下到 Z_stop + offset + insert，再抬回安全高度')
    print(f"     offset={calibration['offset_steps']:+d} 步，"
          f"insert={ELECTRODE_INSERT_STEPS:+d} 步")
    if input('FULL: 确认开始？(y/n): ').strip().lower() != 'y':
        return False

    if not ensure_safe_z(ser):
        return False
    if not move_continuous(ser, {'X': int(target['X']), 'Y': int(target['Y'])}):
        print('FULL: 移动到 P2 失败，已停止')
        ensure_safe_z(ser)
        return False
    if not _rotate_tool(tt, TOOL_DRILL, '钻孔头'):
        ensure_safe_z(ser)
        return False
    if not _z_move(ser, SAFE_Z_STEPS, speed=500):
        print('FULL: 下降到钻孔起始高度失败')
        ensure_safe_z(ser)
        return False

    drill = run_automatic_drill(ser)
    if drill is None or drill.get('status') != 'BREAKTHROUGH' or drill.get('z_stop') is None:
        print('FULL: 钻孔未成功完成，抬到安全高度后停止')
        ensure_safe_z(ser)
        return False
    z_stop = int(drill['z_stop'])
    print(f"FULL: 钻孔完成，Z_stop={z_stop}，钻深={drill.get('depth_steps')} 步")

    if not ensure_safe_z(ser):
        return False
    if not _rotate_tool(tt, TOOL_ELECTRODE, '电极头'):
        ensure_safe_z(ser)
        return False

    electrode_z = int(round(z_stop + float(calibration['offset_steps']) + ELECTRODE_INSERT_STEPS))
    if electrode_z < 0:
        print(f'FULL: 电极目标 Z={electrode_z} 为负，拒绝移动')
        ensure_safe_z(ser)
        return False
    print(f'FULL: 电极目标 Z={electrode_z}')
    if not _z_move(ser, electrode_z, speed=ELECTRODE_TRAVEL_SPEED):
        print('FULL: 电极下降失败，已停止')
        ensure_safe_z(ser)
        return False
    print('FULL: 电极已下到接触 Z')
    if not ensure_safe_z(ser):
        return False
    print('FULL: 完整流程结束')
    return True


"""20261011新增头部视觉识别"""
def run_visual_head(ser):
    """HOME → 视觉位 → 调用 vis_head.py 识别头部四点。"""
    if not goto_vision_position(ser):
        return False

    try:
        points = capture_and_detect_head()
    except Exception as exc:
        print(f'VIS_HEAD: 识别失败，停在视觉位：{exc}')
        return False

    print(f'VIS_HEAD COMPLETE: {points}')
    return True


def cmd_mode(ser, tt):
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
        """20261011新增头部视觉识别"""
        if line.upper() == 'VIS_HEAD':
            run_visual_head(ser)
            continue
        # 钻孔循环：传感器阶跃 -> 停钻 -> 抬升（需要传感器串口可用）
        if line.upper() in ('DC', 'DRILLCYCLE'):
            run_automatic_drill(ser)
            continue
        # 电极接触 Z 标定（无传感器，人工确认接触）
        if line.upper() in ('ECAL', 'ZCAL'):
            run_electrode_contact_test(ser, tt)
            continue
        # 完整流程：P2 -> 钻孔 -> 换电极头 -> 下到接触 Z
        if line.upper() in ('FULL', 'FLOW', 'AUTOFLOW'):
            run_full_sequence(ser, tt)
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
        send(ser, command)
        up = command.upper()
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
    print("  VIS_HEAD                  归零→视觉位→调用vis_head.py识别头部4点")
    print("  DC (DRILLCYCLE)           传感器阶跃停钻→抬升（需XY已在孔位；传感器串口独立）")
    print("  ECAL (ZCAL)              电极接触Z标定：转电极头→每次下探1mm→人工确认→保存offset")
    print("  FULL (FLOW)              完整流程：P2→钻孔→抬升→转电极头→下到接触Z→抬升")
    print("  流程前置: 转盘 TZ→TE；VIS 已生成本次P2目标；ECAL 已有标定")
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
