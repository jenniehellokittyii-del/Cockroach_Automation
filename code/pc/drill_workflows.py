# -*- coding: utf-8 -*-
"""钻孔 D1-D3 流程编排。

职责：
- D1：安全高度 -> 钻孔头 -> 复用 drill_cycle（含 drill_duty）
- D2：安全高度 -> 电极头
- D3：安全高度 -> 电极头 -> 按 Z_STOP + offset + insert 下探
- ECAL：电极接触标定（可选）
- 配置读取、运行状态写入 runtime，不修改 config 初始值。

现场入口是 drill_console.py；motion_client.py 的 DC 也复用本模块。
"""

from __future__ import annotations

import json
import time
from datetime import datetime
from pathlib import Path

import serial_motion as motion
from serial_motion import (
    send, read_response, query_positions, wait_positions, move_continuous,
)
from settings import (
    DRILL_PARAMS_CONFIG, SAFE_Z_CONFIG, DRILL_START_CONFIG,
    ELECTRODE_CALIBRATION_CONFIG, DRILL_STATE_PATH, DRILL_EVENT_LOG_DIR,
)
from force_sensor import ForceSensor, load_config as load_force_sensor_config
from drill_cycle import DrillParams, run_drill_cycle

# 当前会话的钻孔结果；Z_STOP 不写固定配置，D1/DC 会更新。
LAST_DRILL_RESULT = None


# ---------------- 配置 / 状态 ----------------

def _read_json(path, default=None):
    try:
        return json.loads(Path(path).read_text(encoding='utf-8'))
    except Exception:
        return default


def _write_json(path, payload):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding='utf-8')


def load_params():
    """从 config/drill_params.json 构造钻孔参数；缺省时使用安全默认值。"""
    data = _read_json(DRILL_PARAMS_CONFIG, {}) or {}
    allowed = {
        'feed_speed', 'normal_speed', 'drill_duty', 'safe_depth_steps',
        'lift_steps', 'settle_seconds', 'extra_drill_seconds',
        'timeout_seconds',
    }
    kwargs = {k: v for k, v in data.items() if k in allowed}
    kwargs.setdefault('drill_duty', 0)
    kwargs.setdefault('safe_depth_steps', 2500)
    kwargs.setdefault('lift_steps', 2000)
    return DrillParams(**kwargs)


def load_state():
    """config 是初始值，runtime/drill_state.json 是运行后的最新值。"""
    safe = _read_json(SAFE_Z_CONFIG, {}) or {}
    start = _read_json(DRILL_START_CONFIG, {}) or {}
    cal = _read_json(ELECTRODE_CALIBRATION_CONFIG, {}) or {}
    state = {
        'safe_z_steps': int(safe.get('safe_z_steps', 0)),
        'drill_start_z': int(start.get('drill_start_z', 2250)),
        'offset_steps': int(cal.get('offset_steps', 0)),
        'insert_steps': int(cal.get('insert_steps', 0)),
    }
    runtime = _read_json(DRILL_STATE_PATH, {}) or {}
    for key in state:
        if key in runtime:
            state[key] = int(runtime[key])
    return state


def save_state(**changes):
    state = load_state()
    state.update({k: int(v) for k, v in changes.items()})
    _write_json(DRILL_STATE_PATH, state)
    return state


def tools_from_config():
    data = _read_json(DRILL_PARAMS_CONFIG, {}) or {}
    return int(data.get('tool_electrode', 1)), int(data.get('tool_drill', 2))


def ecal_speed():
    data = _read_json(DRILL_PARAMS_CONFIG, {}) or {}
    return int(data.get('ecal_speed', 150)), int(data.get('ecal_step_steps', 100)), int(data.get('ecal_max_steps', 10000))


def d3_speed():
    data = _read_json(DRILL_PARAMS_CONFIG, {}) or {}
    return int(data.get('d3_speed', 500))


# ---------------- 基础动作 ----------------

def _z_move(ser, target, speed=500):
    send(ser, f'SPEED {int(speed)}')
    read_response(ser, timeout=0.5)
    return move_continuous(ser, {'Z': int(target)})


def ensure_safe_z(ser):
    state = load_state()
    safe = state['safe_z_steps']
    pos = query_positions(ser)
    if pos is None:
        print('流程: 读取 POS 失败')
        return False
    if pos['Z'] <= safe:
        print(f'流程: Z={pos["Z"]} 已在安全高度（≤{safe}）')
        return True
    print(f'流程: Z {pos["Z"]} -> 安全高度 {safe}')
    return _z_move(ser, safe, 500)


def _turntable_ready(tt):
    if tt is None:
        print('流程: 转盘未连接')
        return False
    if not getattr(tt, 'zeroed', False):
        print('流程: 转盘零点未建立，请先 TZ')
        return False
    if not getattr(tt, 'enabled', False):
        print('流程: 转盘未使能，请先 TE')
        return False
    return True


def _rotate_tool(tt, tool, name):
    if not _turntable_ready(tt):
        return False
    if int(getattr(tt, 'tool', -1)) == int(tool):
        print(f'流程: 已在{name}（工具{tool}）')
        return True
    print(f'流程: 转盘转到{name}（工具{tool}）')
    tt.goto_tool(int(tool))
    return int(getattr(tt, 'tool', -1)) == int(tool)


class SafeDrillIO:
    """把 ZDOWN 换成相对下行，避免负 Z 时固件 32 位溢出。"""

    REL_DOWN_STEPS = 100000

    def __init__(self, ser):
        self.ser = ser
        self._expect_down_ack = False

    def send(self, line):
        if line.strip().upper() in ('ZDOWN', 'ZD'):
            self._expect_down_ack = True
            line = f'Z {self.REL_DOWN_STEPS}'
        send(self.ser, line)

    def read_response(self, markers=None, timeout=0.3):
        out = read_response(self.ser, markers=markers, timeout=timeout)
        if self._expect_down_ack:
            self._expect_down_ack = False
            marker = f'MOVE Z {self.REL_DOWN_STEPS}'
            if marker in out:
                out = out.replace(marker, 'MOVE Z DOWN CONTINUOUS (REL)')
        return out

    def query_positions(self, timeout=1.0):
        return query_positions(self.ser, timeout=timeout)

    def wait_z(self, target, timeout=20.0):
        return wait_positions(self.ser, {'Z': target}, timeout=timeout)

    def fault(self):
        return motion.motion_fault

    def clear_fault(self):
        motion.motion_fault = None

    def log(self, message):
        print(message)
        motion.log_io('EVT', message)


# ---------------- 钻孔 / 电极 ----------------

def run_automatic_drill(ser, sensor=None, params=None):
    """DC/D1 共用核心：传感器阶跃 -> 停钻 -> 抬升。"""
    global LAST_DRILL_RESULT
    own_sensor = sensor is None
    if own_sensor:
        try:
            sensor = ForceSensor(config=load_force_sensor_config())
            sensor.start()
        except Exception as exc:
            print(f'DC: 传感器启动失败，未开始：{exc}')
            return None
    print(f'DC: 传感器已连接（{sensor.describe()}）')
    print(f'DC: 数据记录 {sensor.csv_path}')
    try:
        io = SafeDrillIO(ser)
        result = run_drill_cycle(io, sensor, params=params or load_params(), log=io.log)
        LAST_DRILL_RESULT = result
        event = result.get('trigger')
        if isinstance(event, dict) and event.get('time') is not None:
            DRILL_EVENT_LOG_DIR.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            path = DRILL_EVENT_LOG_DIR / f'drill_event_{stamp}.csv'
            try:
                rows = sensor.save_window(path, float(event['time']), before=2.0, after=2.0)
                print(f'DC: 触发前后 {rows} 个样本 -> {path}')
            except Exception as exc:
                print(f'DC: 保存触发窗口失败：{exc}')
        print(f"DC: status={result.get('status')} "
              f"Z_stop={result.get('z_stop')} lifted={result.get('lifted')}")
        return result
    finally:
        if own_sensor:
            try:
                sensor.close()
            except Exception:
                pass


def _zstop_usable(drill):
    """D3/ECAL 只接受成功钻孔或人工 ZSTOP。"""
    if not drill or drill.get('z_stop') is None:
        return False
    status = drill.get('status')
    if status == 'MANUAL_INPUT':
        return True
    if status != 'BREAKTHROUGH':
        return False
    if not drill.get('lifted'):
        return False
    if drill.get('drill_off_verified') is not True:
        return False
    if drill.get('safe_returned') is False:
        return False
    return True


def drill_stage(ser, tt, sensor=None):
    """D1：安全高度 -> 钻孔头 -> 移动到钻孔起点 -> 钻孔核心 -> 抬升。"""
    _, tool_drill = tools_from_config()
    if not _turntable_ready(tt):
        return None
    if not ensure_safe_z(ser):
        return None
    if not _rotate_tool(tt, tool_drill, '钻孔头'):
        return None

    state = load_state()
    start_z = state.get('drill_start_z')
    if start_z is None:
        print('D1: 没有 DRILL_START_Z，拒绝下钻')
        return None
    start_z = int(start_z)
    if start_z < 0:
        print(f'D1: DRILL_START_Z={start_z} 为负，拒绝下钻')
        ensure_safe_z(ser)
        return None
    print(f'D1: 安全高度 -> DRILL_START_Z={start_z}')
    if not _z_move(ser, start_z, 500):
        print('D1: 移动到钻孔起点失败')
        ensure_safe_z(ser)
        return None
    pos = query_positions(ser)
    if pos is None or int(pos['Z']) != start_z:
        print(f'D1: 钻孔起点未确认（期望 {start_z}，实际 {pos}）')
        ensure_safe_z(ser)
        return None

    result = run_automatic_drill(ser, sensor=sensor)
    if result is None:
        print('D1: 未执行钻孔')
        ensure_safe_z(ser)
        return None

    result['safe_returned'] = bool(ensure_safe_z(ser))
    result['drill_start_z'] = start_z
    print(f"D1: Z_stop={result.get('z_stop')} status={result.get('status')} "
          f"lifted={result.get('lifted')} safe_returned={result.get('safe_returned')}")
    return result


def swap_stage(ser, tt):
    """D2：安全高度 -> 电极头，只换头。"""
    tool_electrode, _ = tools_from_config()
    if not _turntable_ready(tt):
        return False
    if not ensure_safe_z(ser):
        return False
    return _rotate_tool(tt, tool_electrode, '电极头')


def electrode_stage(ser, tt):
    """D3：安全高度 -> 电极头 -> Z_STOP + offset + insert。"""
    global LAST_DRILL_RESULT
    tool_electrode, _ = tools_from_config()
    if not _turntable_ready(tt):
        return False
    if not LAST_DRILL_RESULT or LAST_DRILL_RESULT.get('z_stop') is None:
        print('D3: 没有钻孔 Z_STOP；先 D1 或输入 ZSTOP <Z>')
        return False
    if not _zstop_usable(LAST_DRILL_RESULT):
        print(f"D3: 当前钻孔结果不可用（status={LAST_DRILL_RESULT.get('status')}，"
              f"lifted={LAST_DRILL_RESULT.get('lifted')}）")
        return False
    state = load_state()
    target = int(round(
        int(LAST_DRILL_RESULT['z_stop'])
        + int(state['offset_steps'])
        + int(state['insert_steps'])
    ))
    print(f"D3: 目标 = Z_STOP({LAST_DRILL_RESULT['z_stop']}) "
          f"+ offset({state['offset_steps']:+d}) + insert({state['insert_steps']:+d}) = {target}")
    if target < 0:
        print('D3: 目标 Z 为负，拒绝移动')
        return False
    if not ensure_safe_z(ser):
        return False
    if not _rotate_tool(tt, tool_electrode, '电极头'):
        return False
    if not _z_move(ser, target, d3_speed()):
        print('D3: 电极下降失败')
        ensure_safe_z(ser)
        return False
    print('D3: 电极到位')
    if not ensure_safe_z(ser):
        print('D3: 抬回安全高度失败')
        return False
    return True


def run_full(ser, tt, sensor=None):
    """FULL/RUN：D1 -> D2 -> D3。D1 未满足成功条件时不继续。"""
    result = drill_stage(ser, tt, sensor=sensor)
    if result is None:
        print('FULL: D1 未完成，停止')
        return False
    if result.get('status') != 'BREAKTHROUGH':
        print(f"FULL: D1 状态 {result.get('status')}，停止")
        return False
    if not result.get('lifted'):
        print('FULL: D1 抬升未确认，停止')
        return False
    if result.get('drill_off_verified') is not True:
        print('FULL: D1 钻头停止未确认，停止')
        return False
    if not result.get('safe_returned'):
        print('FULL: D1 回到安全高度失败，停止')
        return False
    if not swap_stage(ser, tt):
        print('FULL: D2 未完成')
        return False
    if not electrode_stage(ser, tt):
        print('FULL: D3 未完成')
        return False
    print('FULL: 完成')
    return True


def ecal(ser, tt):
    """ECAL：1mm 步进找电极接触，保存 offset = contact - Z_STOP。"""
    global LAST_DRILL_RESULT
    tool_electrode, _ = tools_from_config()
    if not _turntable_ready(tt):
        return False
    if not LAST_DRILL_RESULT or LAST_DRILL_RESULT.get('z_stop') is None:
        print('ECAL: 没有 Z_STOP；先 D1 或 ZSTOP <Z>')
        return False
    if not _zstop_usable(LAST_DRILL_RESULT):
        print(f"ECAL: 当前钻孔结果不可用（status={LAST_DRILL_RESULT.get('status')}）")
        return False
    z_stop = int(LAST_DRILL_RESULT['z_stop'])
    speed, step, max_steps = ecal_speed()
    print(f'ECAL: Z_STOP={z_stop}，每次 {step} 步，最多 {max_steps} 步')
    if input('ECAL: 确认开始？(y/n): ').strip().lower() != 'y':
        return False
    if not ensure_safe_z(ser) or not _rotate_tool(tt, tool_electrode, '电极头'):
        return False
    send(ser, 'DRILL OFF')
    read_response(ser, timeout=0.3)
    send(ser, f'SPEED {speed}')
    read_response(ser, timeout=0.3)
    contact = None
    travelled = 0
    try:
        while travelled < max_steps:
            pos = query_positions(ser)
            if pos is None:
                print('ECAL: 读取 POS 失败')
                break
            expected = pos['Z'] + step
            send(ser, f'Z {step}')
            read_response(ser, timeout=0.3)
            if not wait_positions(ser, {'Z': expected}, timeout=15.0):
                print('ECAL: 下探未到位，停止')
                send(ser, 'STOP')
                read_response(ser, timeout=0.3)
                break
            travelled += step
            pos = query_positions(ser)
            if pos is None:
                break
            current = pos['Z']
            print(f'ECAL: 下探 {travelled} 步，Z={current}，相对 Z_STOP {current - z_stop:+d}')
            answer = input('  接触了吗？y=确认接触, 回车=继续, q=放弃: ').strip().lower()
            if answer == 'q':
                print('ECAL: 已放弃')
                break
            if answer == 'y':
                contact = current
                break
    finally:
        send(ser, f'SPEED 500')
        read_response(ser, timeout=0.3)
        ensure_safe_z(ser)
    if contact is None:
        return False
    offset = int(contact - z_stop)
    save_state(offset_steps=offset)
    print(f'ECAL: 接触 Z={contact}，offset={offset:+d} 已保存到 {DRILL_STATE_PATH}')
    return True


# ---------------- 标定 / Z_STOP 命令 ----------------

def set_safe_z(value):
    save_state(safe_z_steps=int(value))
    print(f'SAFE_Z={int(value)}')


def set_drill_start(value):
    save_state(drill_start_z=int(value))
    print(f'DRILL_START_Z={int(value)}')


def set_offset(value):
    save_state(offset_steps=int(value))
    print(f'OFFSET={int(value)}')


def set_insert(value):
    save_state(insert_steps=int(value))
    print(f'INSERT={int(value)}')


def set_zstop(value):
    global LAST_DRILL_RESULT
    LAST_DRILL_RESULT = {'status': 'MANUAL_INPUT', 'z_stop': int(value)}
    print(f'Z_STOP={int(value)}（本会话）')


def get_zstop():
    return LAST_DRILL_RESULT.get('z_stop') if LAST_DRILL_RESULT else None
