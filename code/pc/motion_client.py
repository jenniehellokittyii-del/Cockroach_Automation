import time
from settings import *
import serial_motion as motion
from serial_motion import (pick_port, open_verified_arduino, send, read_response,
                           normalize_motion_command)
from turntable import Turntable, handle_turntable
from vision_workflows import run_automatic_vision, run_visual_head
from drill_workflows import run_automatic_drill

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
        # 钻孔循环：传感器阶跃 -> 停钻 -> 抬升（需要传感器串口可用）
        if line.upper() in ('DC', 'DRILLCYCLE'):
            run_automatic_drill(ser)
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
            motion.vision_reference_ready = False
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
            motion.vision_reference_ready = 'HOME DONE' in out and 'FAIL' not in out

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
    print("  DC (DRILLCYCLE)           传感器阶跃停钻→抬升（需XY已在孔位；传感器串口独立）")
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


if __name__ == '__main__':
    main()
