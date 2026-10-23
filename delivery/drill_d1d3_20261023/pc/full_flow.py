# -*- coding: utf-8 -*-
"""钻孔调试控制台（从 P2/当前 XY 开始，手动操作 + 一条 FULL 指令）。

设计：
  - 不做取电极、不做视觉；先手动把 XY 移到目标点（P2），再输入 FULL。
  - 控制台手动命令与 motion_client 类似：X/Y/Z/B、G0/M0、POS、HOME、DRILL、E 等。
  - FULL 一条命令执行：安全高度 → 转钻孔头 → 钻孔 → 抬升 → 转电极头
    → 电极按 ECAL 偏移下探 → 抬回安全高度。
  - 现场默认接传感器（阶跃自动停钻）；`--no-sensor` 仅用于离线调试。

Z 的三个量：
  1. SAFE_Z        绝对安全高度（pc/safe_z.json 或 --safe-z；默认 0）
  2. DRILL_START_Z 钻孔起始高度（pc/drill_start_z.json 或 --drill-start-z；钻头尖离样品 1~2mm）
  3. 电极偏移      ECAL: offset = 电极接触 Z - 本次钻孔 Z_stop
                   电极目标 Z = Z_stop + offset + 插入量（insert=0 即只接触，不压）

用法（在“运动控制”目录）：
    python pc\\full_flow.py                 # 现场（有传感器，默认）
    python pc\\full_flow.py --safe-z 0 --drill-start-z 2250 --insert 0

控制台命令：
    X100 / Y-50 / Z200 / B10     相对移动（Z+ 向下）
    G0 X1000 Y2000 Z500          绝对步数
    M0 X50 Y30 Z10               绝对毫米
    POS / SPEED500 / HOME / HX / HY / HZ
    DRILL ON / OFF / STATUS / PWM 50
    TZ / TE / TD / T1 / T2 / T3 / TN / TP
    A [N]                        快速下降（不转钻头，默认 500 步）
    F / S                        读一次力值 / 实时 5 秒
    DC                           只钻孔（当前工具必须是钻孔头）
  SAFE                         查看安全高度；SAFE 0 设置并保存
  DRILLSTART                   查看钻孔起始 Z；DRILLSTART 2250 设置并保存
    DRILLSTART s                 1mm 步进下探标定钻孔起始高度
    ZSTOP <z>                    手动录入本次钻停 Z（本会话）
    CONTACT <z>                  手动录入电极接触 Z → 计算并保存 offset
    OFFSET <steps>               直接设置 offset 并保存
    INSERT <steps>               设置电极额外下压量（0=只接触）
    ECAL                         电极接触标定（1mm 步进，人工确认）
    D1 / DRILLSTAGE              阶段1：安全→转钻孔头→钻孔→抬升
    D2 / SWAP                    阶段2：安全→转电极头（只换头）
    D3 / ELECTRODE               阶段3：电极下探（Z_stop+offset+insert）
    FULL / RUN                   三阶段一条跑完
    E / STOP                     急停（含钻头）
    Q                            退出（自动发 E）

安全：全程有人监护；FULL 前确认 XY 已在 P2、转盘已 TZ/TE、SAFE_Z 下旋转不干涉。
"""

import argparse
import json
import re
import sys
import time
from datetime import datetime
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import motion_client as mc
from force_sensor import ForceSensor, load_config as load_sensor_config

SAFE_Z_FILE = PROJECT_ROOT / 'pc' / 'safe_z.json'
DRILL_START_FILE = PROJECT_ROOT / 'pc' / 'drill_start_z.json'
INSERT_STEPS = 0


def ask(prompt, default='n'):
    text = input(f'{prompt} [{"Y/n" if default == "y" else "y/N"}]: ').strip().lower()
    if not text:
        return default == 'y'
    return text in ('y', 'yes', '是')


def load_json(path):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except Exception:
        return None


def save_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')


def connect():
    port = mc.pick_port()
    ser = mc.open_verified_arduino(port)
    print(f'Arduino 已连接 {port} @ {mc.BAUD}')
    return ser


def try_turntable():
    try:
        tt = mc.Turntable(mc.TURNTABLE_PORT, mc.TURNTABLE_BAUD)
        try:
            tt.enable()
        except Exception as exc:
            print(f'转盘连接了但使能失败：{exc}')
        print(f'转盘已连接 {mc.TURNTABLE_PORT} @ {mc.TURNTABLE_BAUD}')
        return tt
    except Exception as exc:
        print(f'转盘未连接（{exc}）；FULL 换头不可用，其余手动命令可继续')
        return None


def safe_z_value(cli_value=None):
    data = load_json(SAFE_Z_FILE) or {}
    return int(cli_value if cli_value is not None else data.get('safe_z_steps', mc.SAFE_Z_STEPS))


def drill_start_value(cli_value=None):
    if cli_value is not None:
        return int(cli_value)
    data = load_json(DRILL_START_FILE) or {}
    value = data.get('drill_start_z')
    return int(value) if value is not None else None


def print_state(ser, tt, use_sensor, insert_steps):
    pos = mc.query_positions(ser)
    tt_state = '未连接'
    if tt is not None:
        tt_state = f'工具{tt.tool} 零点={tt.zeroed} 使能={tt.enabled}'
    print(f'  POS={pos}  转盘={tt_state}')
    print(f'  SAFE_Z={mc.SAFE_Z_STEPS}  DRILL_START_Z={drill_start_value()}  '
          f'钻孔模式={"传感器" if use_sensor else "手动"}  insert={insert_steps}')


def set_safe_z(ser, text):
    if text:
        try:
            value = int(text)
        except ValueError:
            pos = mc.query_positions(ser)
            if pos is None:
                print('读取 POS 失败')
                return
            value = int(pos['Z'])
        save_json(SAFE_Z_FILE, {'saved_at': datetime.now().isoformat(timespec='seconds'),
                                'safe_z_steps': value})
        mc.SAFE_Z_STEPS = value
        print(f'SAFE_Z 已设为 {value}（{value / 100:.2f} mm）')
    else:
        print(f'SAFE_Z={mc.SAFE_Z_STEPS}（{mc.SAFE_Z_STEPS / 100:.2f} mm）')
        print('设置：SAFE <Z步数>；或先把 Z 手动移到安全高度，再输入 SAFE 后回车')


def set_drill_start(text, ser):
    if text == 's':
        value = approach_drill_start(ser)
        if value is not None:
            print(f'DRILL_START_Z={value}')
        return
    if text:
        try:
            value = int(text)
        except ValueError:
            print('请输入整数步数，或 s 进行步进标定')
            return
        save_json(DRILL_START_FILE, {'saved_at': datetime.now().isoformat(timespec='seconds'),
                                     'drill_start_z': value,
                                     'note': '人工设置（钻头尖距样品约1~2mm）'})
        print(f'DRILL_START_Z 已设为 {value}')
    else:
        value = drill_start_value()
        print(f'DRILL_START_Z={value}' if value is not None else '尚未设置 DRILL_START_Z')


def set_insert(text):
    global INSERT_STEPS
    if text:
        try:
            INSERT_STEPS = int(text)
        except ValueError:
            print('insert 必须是整数步数（正值向下）')
            return
        mc.ELECTRODE_INSERT_STEPS = INSERT_STEPS
        print(f'insert={INSERT_STEPS} 步（{INSERT_STEPS / 100:+.2f} mm；0=只接触）')
    else:
        print(f'insert={INSERT_STEPS} 步（0=只接触）')


def set_zstop(text):
    if not text:
        drill = mc.LAST_DRILL_RESULT
        print(f"Z_stop={drill.get('z_stop') if drill else None}")
        return
    try:
        z_stop = int(text)
    except ValueError:
        print('Z_stop 必须是整数步数')
        return
    mc.LAST_DRILL_RESULT = {
        'status': 'MANUAL_INPUT', 'z_start': None, 'z_stop': z_stop,
        'depth_steps': None,
    }
    print(f'Z_stop={z_stop} 已记录（本会话有效）')


def save_electrode_calibration(z_stop, z_contact, offset):
    payload = {
        'saved_at': datetime.now().isoformat(timespec='seconds'),
        'tool_electrode': mc.TOOL_ELECTRODE,
        'tool_drill': mc.TOOL_DRILL,
        'z_stop': z_stop,
        'z_contact': z_contact,
        'offset_steps': offset,
        'offset_mm': offset / 100.0,
        'insert_steps': INSERT_STEPS,
        'source': 'manual',
    }
    save_json(mc.ELECTRODE_CALIBRATION_PATH, payload)
    print(f'offset={offset:+d} 步（{offset / 100:+.2f} mm）已保存 -> '
          f'{mc.ELECTRODE_CALIBRATION_PATH}')
    print('以后电极目标 Z = 本次钻孔 Z_stop + offset + insert')


def set_contact(text):
    drill = mc.LAST_DRILL_RESULT or {}
    z_stop = drill.get('z_stop')
    if z_stop is None:
        print('没有 Z_stop；先输入 ZSTOP <钻停Z>')
        return
    if not text:
        print('用法: CONTACT <电极接触Z>；或 OFFSET <offset步数>')
        return
    try:
        z_contact = int(text)
    except ValueError:
        print('接触 Z 必须是整数步数')
        return
    save_electrode_calibration(z_stop, z_contact, z_contact - z_stop)


def set_offset(text):
    if not text:
        cal = mc._load_json(mc.ELECTRODE_CALIBRATION_PATH)
        print(f"当前 offset={cal.get('offset_steps') if cal else None}")
        return
    try:
        offset = int(text)
    except ValueError:
        print('offset 必须是整数步数')
        return
    drill = mc.LAST_DRILL_RESULT or {}
    z_stop = drill.get('z_stop')
    z_contact = (z_stop + offset) if z_stop is not None else None
    save_electrode_calibration(z_stop, z_contact, offset)


def approach_drill_start(ser):
    """钻头 1mm 步进下探，人工确认接近表面；保存绝对 Z。"""
    print('钻头将以 1mm 步进下探；每步后 y=已接近表面 / 回车=继续 / q=取消')
    mc.send(ser, 'DRILL OFF')
    mc.read_response(ser, timeout=0.3)
    travelled = 0
    try:
        while travelled < 5000:
            pos = mc.query_positions(ser)
            if pos is None:
                print('读取 POS 失败')
                return None
            if not mc._z_move(ser, pos['Z'] + 100, speed=150):
                print('下探失败')
                return None
            travelled += 100
            pos = mc.query_positions(ser)
            print(f'  已下探 {travelled} 步（{travelled / 100:.1f} mm），'
                  f'当前 Z={pos["Z"] if pos else "?"}')
            text = input('  y=已接近表面 / 回车=继续 / q=取消: ').strip().lower()
            if text == 'q':
                return None
            if text == 'y':
                if pos is None:
                    return None
                save_json(DRILL_START_FILE, {
                    'saved_at': datetime.now().isoformat(timespec='seconds'),
                    'drill_start_z': int(pos['Z']),
                    'note': '钻头尖距样品表面约 1~2mm（人工确认）',
                })
                print(f'  已保存 DRILL_START_Z={pos["Z"]}')
                return int(pos['Z'])
    finally:
        mc.ensure_safe_z(ser)
    return None


def drill_once(ser, use_sensor):
    start_z = drill_start_value()
    if start_z is None:
        print('没有 DRILL_START_Z；请输入绝对 Z，或输入 s 标定')
        text = input('  s=步进标定 / 绝对 Z / 回车取消: ').strip().lower()
        if text == 's':
            start_z = approach_drill_start(ser)
        elif text:
            try:
                start_z = int(text)
                save_json(DRILL_START_FILE, {'drill_start_z': start_z})
            except ValueError:
                print('输入无效')
                return None
        if start_z is None:
            return None
    if start_z < 0:
        print('钻孔起始 Z 为负，拒绝下钻')
        return None
    if not mc.ensure_safe_z(ser):
        return None
    if not mc._z_move(ser, start_z, speed=500):
        print('下降到钻孔起始高度失败')
        mc.ensure_safe_z(ser)
        return None

    if use_sensor:
        result = mc.run_automatic_drill(ser)
        if not result or result.get('status') != 'BREAKTHROUGH':
            print('传感器钻孔未成功')
            mc.ensure_safe_z(ser)
            return None
        print(f"  钻孔完成：Z_start={result.get('z_start')} Z_stop={result.get('z_stop')} "
              f"行程={result.get('depth_steps')} 步")
    else:
        print('手动钻孔：DRILL ON + ZDOWN（1.5mm/s），看到刺穿后按回车停止。')
        print('⚠ 手动模式没有软件限深，请盯住钻头，手边可断电。')
        if not ask('开始手动钻孔？', 'n'):
            return None
        pos = mc.query_positions(ser)
        actual_start = int(pos['Z']) if pos else start_z
        mc.send(ser, 'DRILL ON')
        mc.read_response(ser, timeout=1.0)
        mc.send(ser, 'SPEED 150')
        mc.read_response(ser, timeout=0.3)
        mc.send(ser, 'ZDOWN')
        mc.read_response(ser, timeout=1.0)
        try:
            input('看到刺穿/到目标深度后按回车停止...')
        finally:
            mc.send(ser, 'STOP')
            mc.read_response(ser, timeout=1.0)
            mc.send(ser, 'SPEED 500')
            mc.read_response(ser, timeout=0.3)
        pos = mc.query_positions(ser)
        if pos is None:
            print('停止后读取 POS 失败')
            mc.ensure_safe_z(ser)
            return None
        z_stop = int(pos['Z'])
        travel = z_stop - actual_start
        result = {'status': 'MANUAL', 'z_start': actual_start, 'z_stop': z_stop,
                  'depth_steps': travel, 'lifted': False}
        print(f'  手动停止：Z_start={actual_start} Z_stop={z_stop} '
              f'行程={travel} 步（{travel / 100:.2f} mm）')

    mc.LAST_DRILL_RESULT = result
    if not mc.ensure_safe_z(ser):
        print('抬升安全高度未确认，请人工处理')
    return result


def electrode_once(ser, tt, drill_result, insert_steps):
    if not mc._turntable_ready(tt):
        return False
    z_stop = drill_result.get('z_stop') if drill_result else None
    if z_stop is None:
        print('没有钻孔 Z_stop，无法计算电极目标')
        return False
    if not mc.ensure_safe_z(ser):
        return False
    if not mc._rotate_tool(tt, mc.TOOL_ELECTRODE, '电极头'):
        return False

    calibration = mc._load_json(mc.ELECTRODE_CALIBRATION_PATH)
    if not calibration or 'offset_steps' not in calibration:
        print('没有电极接触标定，进入 ECAL（1mm 步进，人工确认接触）')
        mc.LAST_DRILL_RESULT = drill_result
        if not mc.run_electrode_contact_test(ser, tt):
            mc.ensure_safe_z(ser)
            return False
        calibration = mc._load_json(mc.ELECTRODE_CALIBRATION_PATH)
        if not calibration:
            return False
        offset = int(calibration['offset_steps'])
        print(f'ECAL 完成：offset={offset:+d} 步（{offset / 100:+.2f} mm）')
        print(f'以后电极目标 Z = 新 Z_stop + {offset:+d} + 插入量')
        return True

    offset = int(calibration['offset_steps'])
    target = int(round(z_stop + offset + insert_steps))
    print(f'电极目标 Z = Z_stop({z_stop}) + offset({offset:+d}) + insert({insert_steps:+d}) = {target}')
    print(f'offset 已包含钻头扎入深度和工具高度差；insert=0 表示只接触不压。')
    if target < 0:
        print('目标 Z 为负，拒绝移动')
        mc.ensure_safe_z(ser)
        return False
    if not mc._z_move(ser, target, speed=mc.ELECTRODE_TRAVEL_SPEED):
        print('电极下降失败')
        mc.ensure_safe_z(ser)
        return False
    print('电极已到目标 Z（若仍压住样品，请把 insert 调小或恢复 0）')
    mc.ensure_safe_z(ser)
    return True


def approach_down(ser, steps):
    """A 命令：快速下降 N 步（不转钻头），用于接近样品。"""
    pos = mc.query_positions(ser)
    if pos is None:
        print('读取 POS 失败，未接近')
        return
    print(f'接近：Z={pos["Z"]} -> 下降 {steps} 步（{steps / 100:.2f} mm），钻头不转')
    if not ask('确认执行接近？', 'n'):
        return
    if not mc._z_move(ser, pos['Z'] + steps, speed=500):
        print('接近失败/未确认')
        return
    print(f'接近完成：{mc.query_positions(ser)}')


def read_force_once():
    try:
        sensor = ForceSensor(config=load_sensor_config())
        sensor.start()
        deadline = time.monotonic() + 2.0
        while sensor.latest() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        sample = sensor.latest()
        if sample is None:
            print(f'力传感器无数据（{sensor.last_error}）')
        else:
            print(f'力值: monotonic={sample[0]:.3f} raw={sample[1]} value={sample[2]:.3f}')
        sensor.close()
    except Exception as exc:
        print(f'力传感器读取失败：{exc}')


def show_force_live(seconds=5.0):
    try:
        sensor = ForceSensor(config=load_sensor_config())
        sensor.start()
        values = []
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            sample = sensor.latest()
            if sample is not None:
                values.append(sample[1])
                print(f'\r  raw={sample[1]:>10}  value={sample[2]:>12.3f}   ',
                      end='', flush=True)
            time.sleep(0.15)
        print()
        if values:
            print(f'  min={min(values)} max={max(values)} 变化={max(values) - min(values)}')
        sensor.close()
    except Exception as exc:
        print(f'力传感器读取失败：{exc}')


def drill_stage(ser, tt, use_sensor):
    """阶段1 D1：安全高度 → 转钻孔头 → 下到起始 Z → 钻孔 → 抬升。"""
    if tt is None or not mc._turntable_ready(tt):
        print('D1: 转盘未 TZ/TE，无法换头')
        return None
    if not mc.ensure_safe_z(ser):
        return None
    if not mc._rotate_tool(tt, mc.TOOL_DRILL, '钻孔头'):
        return None
    result = drill_once(ser, use_sensor)
    if not result:
        print('D1: 钻孔未完成')
        return None
    if not mc.ensure_safe_z(ser):
        print('D1: 抬升未确认，请人工处理')
    return result


def swap_stage(ser, tt):
    """阶段2 D2：安全高度 → 转电极头（只换头，不下探）。"""
    if tt is None or not mc._turntable_ready(tt):
        print('D2: 转盘未 TZ/TE')
        return False
    if not mc.ensure_safe_z(ser):
        return False
    return mc._rotate_tool(tt, mc.TOOL_ELECTRODE, '电极头')


def electrode_stage(ser, tt, insert_steps):
    """阶段3 D3：用电极头按 Z_stop + offset + insert 下探，再抬升。"""
    drill = mc.LAST_DRILL_RESULT
    if not drill or drill.get('z_stop') is None:
        print('D3: 没有钻孔 Z_stop；先跑 D1 或输入 ZSTOP <z>')
        return False
    if tt is None or not mc._turntable_ready(tt):
        print('D3: 转盘未 TZ/TE')
        return False
    return electrode_once(ser, tt, drill, insert_steps)


def run_full(ser, tt, use_sensor):
    """一条指令：钻孔 → 换电极 → 下探（从当前 XY 开始，不移动 XY）。"""
    if tt is None or not mc._turntable_ready(tt):
        print('FULL: 转盘未 TZ/TE，无法换头；先输入 TZ、TE')
        return
    pos = mc.query_positions(ser)
    print(f'FULL: 从当前 POS={pos} 开始（不会移动 XY）')
    print(f"  1) 抬到 SAFE_Z={mc.SAFE_Z_STEPS}")
    print(f'  2) 转钻孔头（工具{mc.TOOL_DRILL}）→ 下到 DRILL_START_Z={drill_start_value()}')
    print(f'  3) {"传感器" if use_sensor else "手动"}钻孔 → 记录 Z_stop → 抬到安全高度')
    print(f'  4) 转电极头（工具{mc.TOOL_ELECTRODE}）→ Z_stop + offset + insert({INSERT_STEPS})')
    print('  5) 抬回安全高度')
    if not ask('确认开始 FULL？', 'n'):
        return

    drill_result = drill_stage(ser, tt, use_sensor)
    if not drill_result:
        print('FULL: 阶段1（钻孔）未完成，结束')
        return
    if not swap_stage(ser, tt):
        print('FULL: 阶段2（换电极头）未完成，结束')
        return
    if not electrode_stage(ser, tt, INSERT_STEPS):
        print('FULL: 阶段3（电极下探）未完成，结束')
        return
    print('FULL: 结束')


def manual_command(ser, tt, line):
    """与 motion_client 类似的手动命令。"""
    up = line.upper()
    if up in ('HOME', 'HX', 'HY', 'HZ'):
        mc.send(ser, up)
        markers = {'HOME': ('HOME DONE', 'FAIL'), 'HX': ('HOMED X', 'FAIL'),
                   'HY': ('HOMED Y', 'FAIL'), 'HZ': ('HOMED Z', 'FAIL')}
        out = mc.read_response(ser, markers=markers[up], timeout=120)
        print(out, end='')
        return
    if up in ('E', 'STOP'):
        mc.send(ser, 'STOP')
        print(mc.read_response(ser, timeout=1.0), end='')
        return
    if up.startswith('T'):
        if tt is None:
            print('转盘未连接，T 指令不可用')
            return
        if not mc.handle_turntable(tt, line):
            print('未知转盘指令（TZ / TE / TD / T1 / T2 / T3 / TN / TP）')
        return
    try:
        command = mc.normalize_motion_command(line)
    except ValueError as exc:
        print(f'输入错误：{exc}')
        return
    mc.send(ser, command)
    print(mc.read_response(ser, timeout=0.5), end='')


def print_help():
    print("""
================ 钻孔调试控制台 ================
手动（与 motion_client 相同）：
  X100 / Y-50 / Z200 / B10     相对移动（Z+ 向下）
  G0 X1000 Y2000 Z500          绝对步数
  M0 X50 Y30 Z10               绝对毫米
  POS / SPEED500 / HOME / HX / HY / HZ
  DRILL ON / OFF / STATUS / PWM 50
  TZ / TE / TD / T1 / T2 / T3 / TN / TP
  A [N]                        快速下降（不转钻头，默认 500 步）
  F / S                        读一次力值 / 实时 5 秒
  DC                           只钻孔（当前工具必须是钻孔头）
  E / STOP                     急停（含钻头）
调试：
  SAFE                         查看安全高度；SAFE 0 设置并保存
  DRILLSTART                   查看钻孔起始 Z；DRILLSTART 2250 设置并保存
  DRILLSTART s                 1mm 步进下探标定
  ZSTOP <z>                    手动录入本次钻停 Z（本会话）
  CONTACT <z>                  手动录入电极接触 Z → 计算并保存 offset
  OFFSET <steps>               直接设置 offset 并保存
  INSERT <steps>               设置电极额外下压量（0=只接触）
  ECAL                         电极接触标定（1mm 步进）
  D1 / DRILLSTAGE              阶段1：安全→转钻孔头→钻孔→抬升
  D2 / SWAP                    阶段2：安全→转电极头（只换头）
  D3 / ELECTRODE               阶段3：电极下探（Z_stop+offset+insert）
  FULL / RUN                   三阶段一条跑完
  Q                            退出（自动发 E）
================================================
""")


def main():
    parser = argparse.ArgumentParser(description='钻孔调试控制台（从 P2 开始，手动 + FULL）')
    parser.add_argument('--safe-z', type=int, help='安全高度步数（覆盖 pc/safe_z.json）')
    parser.add_argument('--drill-start-z', type=int, help='钻孔起始 Z（覆盖 pc/drill_start_z.json）')
    parser.add_argument('--insert', type=int, default=mc.ELECTRODE_INSERT_STEPS,
                        help=f'电极额外下压步数（默认 {mc.ELECTRODE_INSERT_STEPS}，0=只接触）')
    parser.add_argument('--no-sensor', action='store_true', help='无传感器：手动停钻 + ECAL 人工接触')
    parser.add_argument('--serial-port', help='Arduino 串口（默认自动识别）')
    args = parser.parse_args()
    global INSERT_STEPS
    INSERT_STEPS = int(args.insert)

    safe_z = safe_z_value(args.safe_z)
    mc.SAFE_Z_STEPS = safe_z
    mc.ELECTRODE_INSERT_STEPS = INSERT_STEPS
    if args.drill_start_z is not None:
        save_json(DRILL_START_FILE, {'drill_start_z': int(args.drill_start_z)})

    port = args.serial_port or mc.pick_port()
    try:
        ser = mc.open_verified_arduino(port)
    except RuntimeError as exc:
        print(f'Arduino 连接失败：{exc}')
        return
    print(f'Arduino 已连接 {port} @ {mc.BAUD}')
    tt = try_turntable()
    use_sensor = not args.no_sensor

    print(f'SAFE_Z={mc.SAFE_Z_STEPS}  DRILL_START_Z={drill_start_value()}  '
          f'模式={"传感器" if use_sensor else "手动"}  insert={INSERT_STEPS}')
    print('本脚本从当前 XY 开始；请先手动把 XY 移到 P2，再输入 FULL。')
    print_help()

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
                print_help()
                continue
            if up in ('FULL', 'RUN'):
                run_full(ser, tt, use_sensor)
                continue
            if up in ('D1', 'DRILLSTAGE'):
                drill_stage(ser, tt, use_sensor)
                continue
            if up in ('D2', 'SWAP'):
                swap_stage(ser, tt)
                continue
            if up in ('D3', 'ELECTRODE', 'ELE'):
                electrode_stage(ser, tt, INSERT_STEPS)
                continue
            if up == 'DC':
                drill_once(ser, use_sensor)
                continue
            match = re.fullmatch(r'A\s*(\d*)', up)
            if match:
                steps = int(match.group(1)) if match.group(1) else 500
                approach_down(ser, steps)
                continue
            if up == 'F':
                read_force_once()
                continue
            if up == 'S':
                show_force_live()
                continue
            if up == 'ECAL':
                if tt is None or not mc._turntable_ready(tt):
                    print('ECAL: 转盘未 TZ/TE')
                    continue
                mc.run_electrode_contact_test(ser, tt)
                continue
            if up == 'SAFE' or up.startswith('SAFE '):
                set_safe_z(ser, line[4:].strip())
                continue
            if up == 'DRILLSTART' or up.startswith('DRILLSTART '):
                set_drill_start(line[10:].strip(), ser)
                continue
            if up == 'ZSTOP' or up.startswith('ZSTOP '):
                set_zstop(line[5:].strip())
                continue
            if up == 'CONTACT' or up.startswith('CONTACT '):
                set_contact(line[7:].strip())
                continue
            if up == 'OFFSET' or up.startswith('OFFSET '):
                set_offset(line[6:].strip())
                continue
            if up == 'INSERT' or up.startswith('INSERT '):
                set_insert(line[6:].strip())
                continue
            manual_command(ser, tt, line)
    finally:
        try:
            mc.send(ser, 'E')
            mc.read_response(ser, timeout=0.5)
        except Exception:
            pass
        if tt is not None:
            tt.close()
        ser.close()
        print('已退出（E 已发送）')


if __name__ == '__main__':
    main()
