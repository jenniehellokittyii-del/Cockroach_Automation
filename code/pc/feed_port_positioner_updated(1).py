# -*- coding: utf-8 -*-
"""12 个供料口的独立定位程序。

运行：
    python pc/feed_port_positioner.py
    python pc/feed_port_positioner.py --port COM3

程序连接后首先执行 HOME（Z → Y → X）。归位成功且点位坐标通过检查后，
输入 1～12，程序依次执行：
    1. 从当前 Z 坐标向上抬升 1000 步（Z = 当前 Z - 1000）
    2. X 双丝杆和 Y 轴移动到目标绝对坐标
    3. Z 轴移动到目标绝对坐标

12 个圆圈坐标为逐点实测的绝对坐标（每点 X/Y/Z 独立），存储在 FEED_PORTS。

交互命令：1～12 定位；P 看当前位置；JOG 进入方向键点动微调；REF 打印当前
位置并生成可直接回填 FEED_PORTS 的代码行；ALIGN 对齐 X1/X2；
HOME 重新归位；Q 退出。

本文件不修改 motion_client.py，也不控制 B 轴和钻头。
"""

from __future__ import annotations

import argparse
import re
import sys
import time
from dataclasses import dataclass

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD = 115200
EXPECTED_FIRMWARE_ID = "FW motion-bootpos-p2-20260930"
SAFE_LIFT_STEPS = 1000
MOVE_TIMEOUT_SECONDS = 30.0
HOME_TIMEOUT_SECONDS = 120.0

# 点动每按一下走的步数（想更细就调小；本程序不控制 B 轴）。
JOG_STEP = {"X": 50, "Y": 50, "Z": 50}

# 当前设备中的转盘 USB-CAN 端口，自动选口时不能把它当作 Arduino。
TURNTABLE_PORT = "COM10"
ARDUINO_USB_IDS = {
    (0x1A86, 0x7523),  # CH340
    (0x2341, 0x0010),  # Arduino Mega 2560
    (0x2341, 0x0042),
    (0x2A03, 0x0010),
    (0x2A03, 0x0042),
}


@dataclass(frozen=True)
class FeedPort:
    x: int
    y: int
    z: int


# X 表示 X1/X2 双丝杆的共同绝对坐标；B 轴始终不动。
# 2026-10-16 根据实测 BOOTPOS 将旧临时坐标转换到 HOME 坐标系：
#   HOME坐标 = 旧坐标 + (X=426, Y=-1002, Z=11565)
# 换算依据：第一次运行前机器停在旧点位12，随后失败位置为
# 旧点位12 + (80, -810, -1000)，该位置下一次 HOME 实测
# BOOTPOS=(1386, -2852, 10185)。
#
# 2026-10-28 逐点实测，改为 12 个绝对坐标直接存储（每点 X/Y/Z 独立）。
# X 取 X1（X2 仅跟随、无独立限位）；左列 X、各点 Z 均有差异，故不再用
# 统一的“参考圆 + 相对偏移”模型。夹具头部（上边缘）的 6、7 号已改到尾部。
FEED_PORTS = {
    1:  FeedPort(456,  -2037, 11685),   # 左下角
    2:  FeedPort(406,  -1787, 11685),   # 左列第 2 圆
    3:  FeedPort(406,  -1537, 11685),   # 左列第 3 圆
    4:  FeedPort(356,  -1287, 11685),   # 左列第 4 圆
    5:  FeedPort(396,  -1055, 11685),   # 左上角
    6:  FeedPort(706,  -2037, 11685),   # 尾部中间左
    7:  FeedPort(1006, -2037, 11685),   # 尾部中间右
    8:  FeedPort(1256, -2037, 11685),   # 右下角
    9:  FeedPort(1256, -1787, 11685),   # 右列第 2 圆（自下）
    10: FeedPort(1256, -1537, 11685),   # 右列第 3 圆
    11: FeedPort(1256, -1287, 11685),   # 右列第 4 圆
    12: FeedPort(1256, -1037, 11685),   # 右上角
}

POS_RE = re.compile(
    r"POS X1=(-?\d+) X2=(-?\d+) Y=(-?\d+) Z=(-?\d+) B=(-?\d+)"
)
FAULT_MARKERS = ("LIMIT_STOP", "HOME FAIL", "ERR HOME_REQUIRED", "ERR BUSY")


def choose_arduino_port() -> str:
    """自动选择唯一的 Arduino；不唯一时让操作者选择。"""
    if list_ports is None:
        raise RuntimeError("缺少 pyserial，请先运行: pip install pyserial")
    all_ports = list(list_ports.comports())
    if not all_ports:
        raise RuntimeError("没有检测到串口，请确认 Arduino 已插上 USB")

    candidates = []
    for port in all_ports:
        description = (port.description or "").lower()
        if port.device.upper() == TURNTABLE_PORT.upper():
            continue
        if "蓝牙" in description or "bluetooth" in description:
            continue
        candidates.append(port)
    if not candidates:
        candidates = all_ports

    likely = [
        port
        for port in candidates
        if (port.vid, port.pid) in ARDUINO_USB_IDS
        or "arduino" in (port.description or "").lower()
        or "ch340" in (port.description or "").lower()
    ]
    if len(likely) == 1:
        selected = likely[0]
        print(f"自动识别 Arduino: {selected.device} - {selected.description}")
        return selected.device

    print("检测到以下可用串口：")
    for index, port in enumerate(candidates):
        print(f"  [{index}] {port.device} - {port.description}")
    while True:
        selected = input(f"选择 Arduino 串口序号 [0-{len(candidates) - 1}]: ").strip()
        if selected.isdigit() and 0 <= int(selected) < len(candidates):
            return candidates[int(selected)].device
        print("输入无效，请重新输入。")


class MotionController:
    """供料口程序所需的最小串口运动接口。"""

    def __init__(self, port: str):
        self.port = port
        self.ser = None
        self.fault = None
        self.home_complete = False
        self.home_position = None

    def connect(self) -> None:
        if serial is None:
            raise RuntimeError("缺少 pyserial，请先运行: pip install pyserial")
        try:
            self.ser = serial.Serial(self.port, BAUD, timeout=1)
        except serial.SerialException as exc:
            raise RuntimeError(
                f"无法打开 {self.port}；请关闭 Arduino 串口监视器和其他运动程序"
            ) from exc

        # Mega2560 打开串口时会复位；等待完整启动握手。
        time.sleep(2.0)
        startup = self.ser.read(self.ser.in_waiting).decode("utf-8", "replace")
        if startup:
            print(startup, end="")
        if "READY" not in startup or EXPECTED_FIRMWARE_ID not in startup:
            self.close()
            raise RuntimeError(
                f"{self.port} 未返回指定运动固件握手（{EXPECTED_FIRMWARE_ID}）"
            )

        # 清掉复位前可能残留的运动目标，同时确保钻头停止。
        self.send("E")
        stopped = self.read_response(markers=("STOP E",), timeout=1.0)
        if stopped:
            print(stopped, end="")
        if "STOP E" not in stopped:
            self.close()
            raise RuntimeError("控制板未确认安全停止，拒绝定位")

    def close(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def send(self, command: str) -> None:
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("串口尚未连接")
        self.ser.write((command + "\n").encode("utf-8"))
        self.ser.flush()

    def _record_fault(self, response: str) -> None:
        for marker in FAULT_MARKERS:
            if marker in response:
                self.fault = response.strip()
                return

    def read_response(self, markers=(), timeout: float = 0.3) -> str:
        response = ""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            if waiting:
                response += self.ser.read(waiting).decode("utf-8", "replace")
                self._record_fault(response)
                if markers and any(marker in response for marker in markers):
                    # 标记后继续短暂接收，避免截断 HOME 的 BOOTPOS 后续行。
                    deadline = time.monotonic() + 0.15
                else:
                    # 收到内容后再留一点时间，以接收同一批回复的剩余部分。
                    deadline = time.monotonic() + timeout
            else:
                time.sleep(0.01)
        return response

    def query_position(self, timeout: float = 1.0):
        self.send("POS")
        response = ""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting
            if waiting:
                response += self.ser.read(waiting).decode("utf-8", "replace")
                self._record_fault(response)
                matches = list(POS_RE.finditer(response))
                if matches:
                    x1, x2, y, z, b = map(int, matches[-1].groups())
                    return {"X": x1, "X2": x2, "Y": y, "Z": z, "B": b}
            else:
                time.sleep(0.01)
        return None

    def home_all(self) -> bool:
        """执行 Z、Y、X 全轴归位，并确认归位后的实时坐标。"""
        self.fault = None
        self.home_complete = False
        self.home_position = None
        print("\n开始自动归位：Z → Y → X")
        self.send("HOME")
        response = self.read_response(
            markers=("HOME DONE", "HOME FAIL"),
            timeout=HOME_TIMEOUT_SECONDS,
        )
        if response:
            print(response, end="")
        if "HOME DONE" not in response or "HOME FAIL" in response:
            print("自动归位失败，禁止执行供料口定位。")
            self.emergency_stop()
            return False

        # HOME DONE 表示固件已解除归位故障锁；清掉上一次客户端故障记录后复核。
        self.fault = None
        position = self.query_position()
        if position is None:
            print("归位后无法读取 POS，禁止执行供料口定位。")
            self.emergency_stop()
            return False
        if position["X"] != position["X2"]:
            print(f"归位后 X1/X2 不一致: {_format_position(position)}")
            self.emergency_stop()
            return False

        self.home_complete = True
        self.home_position = position
        print(f"自动归位完成: {_format_position(position)}")
        return True

    def emergency_stop(self) -> None:
        if self.ser is None or not self.ser.is_open:
            return
        try:
            self.send("E")
            response = self.read_response(markers=("STOP E",), timeout=0.5)
            if response:
                print(response, end="")
        except Exception as exc:
            print(f"发送急停失败: {exc}")

    def move_absolute(self, targets, timeout: float = MOVE_TIMEOUT_SECONDS) -> bool:
        """发送一段绝对运动，并用实时 POS 确认所有目标轴到位。"""
        allowed_axes = ("X", "Y", "Z")
        if not targets or any(axis not in allowed_axes for axis in targets):
            raise ValueError(f"无效的运动目标: {targets}")

        self.fault = None
        command = "G0 " + " ".join(
            f"{axis}{int(targets[axis])}" for axis in allowed_axes if axis in targets
        )
        self.send(command)
        response = self.read_response(timeout=0.15)
        if response:
            print(response, end="")
        if self.fault or "ERR" in response:
            self.emergency_stop()
            return False

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            position = self.query_position()
            if self.fault:
                print(f"运动故障: {self.fault}")
                self.emergency_stop()
                return False
            if position is not None:
                reached = all(position[axis] == target for axis, target in targets.items())
                if "X" in targets:
                    reached = reached and position["X2"] == targets["X"]
                if reached:
                    return True
            time.sleep(0.05)

        print(f"等待到位超时，目标={targets}")
        self.emergency_stop()
        return False


def _format_position(position) -> str:
    return (
        f"X1={position['X']} X2={position['X2']} "
        f"Y={position['Y']} Z={position['Z']} B={position['B']}"
    )


def move_to_feed_port(controller: MotionController, port_number: int) -> bool:
    """按照“抬 Z → 走 XY → 下 Z”的顺序定位一个供料口。"""
    if port_number not in FEED_PORTS:
        raise ValueError("供料口编号必须是 1～12")
    if not controller.home_complete or controller.home_position is None:
        print("本次运行尚未成功归位，禁止执行点位移动。")
        return False

    target = FEED_PORTS[port_number]
    current = controller.query_position()
    if current is None:
        print("无法读取当前位置，未执行移动。")
        return False
    if controller.fault:
        print(f"当前存在运动故障，未执行移动: {controller.fault}")
        return False
    if current["X"] != current["X2"]:
        print(f"X1/X2 坐标不一致，未执行移动: {_format_position(current)}")
        return False

    expected = {"X": target.x, "Y": target.y, "Z": target.z}
    if all(current[axis] == value for axis, value in expected.items()):
        print(f"已经位于供料口 {port_number}: {_format_position(current)}")
        return True

    # HOME 后已经处于 Z 轴最高安全位置，第一次移动无需再越过上限位。
    # 后续点位切换仍执行当前 Z-1000；接近上限时钳制在归位坐标。
    home_safe_z = controller.home_position["Z"]
    safe_z = max(current["Z"] - SAFE_LIFT_STEPS, home_safe_z)
    print(
        f"供料口 {port_number}: 当前 {_format_position(current)}\n"
        f"  1/3 Z 抬升到 {safe_z}"
    )
    if not controller.move_absolute({"Z": safe_z}):
        print("Z 抬升失败，已取消后续移动。")
        return False

    print(f"  2/3 X/Y 移动到 X={target.x}, Y={target.y}")
    if not controller.move_absolute({"X": target.x, "Y": target.y}):
        print("X/Y 移动失败，Z 保持在安全高度，已取消下降。")
        return False

    print(f"  3/3 Z 移动到 {target.z}")
    if not controller.move_absolute({"Z": target.z}):
        print("Z 到目标高度失败，本次定位未完成。")
        return False

    final_position = controller.query_position()
    if final_position is None or any(
        final_position[axis] != value for axis, value in expected.items()
    ) or final_position["X2"] != target.x:
        print(f"最终位置复核失败: {final_position}")
        controller.emergency_stop()
        return False

    print(f"供料口 {port_number} 已到达: {_format_position(final_position)}")
    return True


def print_feed_ports() -> None:
    print("\n供料口 HOME 绝对坐标（步）：")
    for number, point in FEED_PORTS.items():
        print(
            f"  {number:>2}: X1={point.x:>4} X2={point.x:>4} "
            f"Y={point.y:>5} Z={point.z:>4} B=0"
        )


def validate_feed_ports_after_home(home_position) -> bool:
    """防止把临时启动零点下记录的旧 Z 坐标用于 HOME 坐标系。"""
    invalid = [
        number
        for number, point in FEED_PORTS.items()
        if point.z < home_position["Z"]
    ]
    if not invalid:
        return True

    numbers = "、".join(map(str, invalid))
    print(
        "\n点位坐标检查失败：归位后 Z 轴向下为正，"
        f"归位位置 Z={home_position['Z']}，但点位 {numbers} 的 Z 坐标更小。\n"
        "这些点位是在旧的临时启动零点下记录的，不能直接用于 HOME 坐标系。\n"
        "请在本次 HOME 后重新逐点测量，更新 FEED_PORTS。"
    )
    return False


def _jog_mode(controller: MotionController) -> None:
    """方向键点动（仅 X/Y/Z），用于把吸嘴精确对准圆圈后读参考坐标。"""
    try:
        import msvcrt
    except ImportError:
        print("点动仅在 Windows 终端可用。")
        return

    print("\n=== 点动模式 ===")
    print("  ↑/↓ = X 轴（前/后）")
    print("  ←/→ = Y 轴（左/右）")
    print("  W/S  = Z 轴（上/下）   W=上  S=下")
    print("  q = 退出点动\n")
    while True:
        ser = controller.ser
        while ser.in_waiting:
            ser.read(ser.in_waiting)
        if not msvcrt.kbhit():
            time.sleep(0.02)
            continue
        ch = msvcrt.getwch()
        if ch in ("\xe0", "\x00"):
            ch = msvcrt.getwch()
        else:
            ch = ch.lower()
        if ch == "q":
            break
        command = {
            "H": f"X{JOG_STEP['X']}",    # ↑ X+
            "P": f"X-{JOG_STEP['X']}",   # ↓ X-
            "K": f"Y{JOG_STEP['Y']}",    # ← Y+
            "M": f"Y-{JOG_STEP['Y']}",   # → Y-
            "w": f"Z-{JOG_STEP['Z']}",   # Z 上
            "s": f"Z{JOG_STEP['Z']}",    # Z 下
        }.get(ch)
        if command:
            controller.send(command)


def _print_reference(controller: MotionController) -> None:
    """打印当前位置，并给出可直接复制回填 FEED_PORTS 的代码行。"""
    position = controller.query_position()
    if position is None:
        print("读取位置失败")
        return
    print(_format_position(position))
    print(f"FeedPort({position['X']}, {position['Y']}, {position['Z']})")


def _align_x(controller: MotionController) -> None:
    """对齐 X1/X2 计数：X2 无独立限位，点动后偶发与 X1 差 1 步。"""
    controller.send("ALIGN")
    response = controller.read_response(markers=("ALIGN",), timeout=1.0)
    if response:
        print(response, end="")
    position = controller.query_position()
    if position is None:
        print("ALIGN 后读取位置失败")
        return
    print(_format_position(position))
    if position["X"] != position["X2"]:
        print("X1/X2 仍不一致，建议执行 HOME 重新归位。")
    else:
        print("X1/X2 已对齐。")


def run_interactive(controller: MotionController) -> None:
    print_feed_ports()
    print("\n命令：1～12=定位  P=当前位置  JOG=点动  REF=读参考坐标  "
          "ALIGN=X1/X2对齐  HOME=重新归位  Q=退出。")
    while True:
        choice = input("供料口 >>> ").strip().lower()
        if choice in ("q", "quit", "exit"):
            return
        if choice in ("p", "pos"):
            position = controller.query_position()
            print(_format_position(position) if position else "读取位置失败")
            continue
        if choice in ("jog", "j"):
            _jog_mode(controller)
            continue
        if choice in ("ref", "r"):
            _print_reference(controller)
            continue
        if choice in ("align", "a"):
            _align_x(controller)
            continue
        if choice in ("home", "h"):
            controller.home_all()
            continue
        if not choice.isdigit() or int(choice) not in FEED_PORTS:
            print("输入无效，请输入 1～12、P、JOG、REF、ALIGN、HOME 或 Q。")
            continue
        move_to_feed_port(controller, int(choice))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="12 个供料口安全定位程序")
    parser.add_argument("--port", help="Arduino 串口，例如 COM3；不填写则自动识别")
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        port = args.port or choose_arduino_port()
        controller = MotionController(port)
        controller.connect()
        print(f"\n已连接 {port} @ {BAUD}")
        if not controller.home_all():
            return 1
        position = controller.home_position
        if not validate_feed_ports_after_home(position):
            return 1
        run_interactive(controller)
        return 0
    except (RuntimeError, OSError) as exc:
        print(f"程序停止: {exc}")
        return 1
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在急停。")
        return 130
    finally:
        if "controller" in locals():
            controller.emergency_stop()
            controller.close()


if __name__ == "__main__":
    sys.exit(main())
