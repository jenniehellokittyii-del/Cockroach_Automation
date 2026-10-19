from pathlib import Path
from datetime import datetime

PROJECT_ROOT = Path(__file__).resolve().parents[2]
LOG_PATH = PROJECT_ROOT / 'runtime' / 'motion_logs' / (datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '.log')
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
# 点位2加入独立 X 补偿后，从视觉位的正常行程可达到约 830 步；
# 保留少量识别波动余量，同时继续拦截点位1从视觉位直接移动约 1000 步的情况。
VISION_MAX_XY_MOVE_STEPS = 850
VISION_HEAD_MAX_XY_MOVE_STEPS = 1000
VISION_PIXEL_CALIBRATION = PROJECT_ROOT / 'config' / 'xy_pixel_calibration.json'
VISION_TOOL_CALIBRATION = PROJECT_ROOT / 'config' / 'manual_left_p2_20260909.json'
VISION_POINT_COMPENSATION = PROJECT_ROOT / 'config' / 'vision_point_compensation.json'
VISION_HEAD_POINT_COMPENSATION = PROJECT_ROOT / 'config' / 'vision_head_point_compensation.json'
VISION_OUTPUT_ROOT = PROJECT_ROOT / 'runtime' / 'vision_output' / 'auto'
VISION_HEAD_OUTPUT_ROOT = PROJECT_ROOT / 'runtime' / 'vision_output' / 'head_auto'
VISION_SHOW_RESULT = True

# ---------------- 钻孔循环（DC 命令）配置 ----------------
FORCE_SENSOR_CONFIG = PROJECT_ROOT / 'config' / 'force_sensor.json'
DRILL_EVENT_LOG_DIR = PROJECT_ROOT / 'runtime' / 'motion_logs'

# ---------------- 转盘电机（USB-CAN）配置 ----------------
TURNTABLE_PORT = 'COM10'   # USB-CAN 模块在设备管理器里的 COM 号
TURNTABLE_BAUD = 921600
TURNTABLE_KP = 20          # 位置刚度
TURNTABLE_KD = 1.0         # 速度阻尼
TURNTABLE_SPEED = 0.6      # 转盘角速度 (rad/s)，觉得慢就调大
TURNTABLE_STARTUP_ENABLED = True  # 当前阶段先完成自动视觉XY，暂不自动连接/换头


