# -*- coding: utf-8 -*-
"""钻孔循环状态机：传感器阶跃 -> 停钻停 Z -> 向上抬升固定距离。

机制（全部复用固件已有指令，不改固件）：
  - 进给：`SPEED <进给>` + `ZDOWN`（Z+ 为下，固件持续下行，E/STOP/Zmax 限位即停）
  - 停止：`STOP`（固件同一处理里先关钻头，再立即清目标停 Z）
  - 抬升：`SPEED <恢复>` + `Z -<A>`（相对上移；Z- 为上）

安全约束：
  - 任何异常路径统一为 `STOP -> DRILL STATUS 复核 -> 记录位置 -> 恢复 SPEED -> 抬升 A -> 上报`
  - `SPEED` 是全轴速度，钻孔慢速后必须恢复（本模块固定流程，测试有断言）
  - 安全限深 `D_safe` 是硬上限；Zmax 限位（D27）是物理兜底（表现为 LIMIT_STOP）

与硬件解耦：io / sensor / detector 均可注入；本地测试见 tests/test_drill_cycle.py。
"""

from __future__ import annotations

import time
from dataclasses import asdict, dataclass, field, is_dataclass

from step_detect import StepDetector

__all__ = ['DrillParams', 'run_drill_cycle']


@dataclass
class DrillParams:
    """钻孔参数（全部现场可调，单位见注释）。"""
    feed_speed: float = 20.0            # 钻孔进给速度（步/秒），0.2 mm/s 首测保守；跑通后可 30~40
    normal_speed: float = 500.0         # 钻完恢复的全轴速度（固件默认）
    safe_depth_steps: int = 300         # 安全限深（步），Z+ 为下；3 mm 初值（按实测刺穿深度+1mm 余量调）
    lift_steps: int = 1000              # 触发后抬升距离（步）；10 mm
    timeout_seconds: float = 60.0       # 总超时：未检测到阶跃则中止
    pos_poll_interval: float = 0.1      # POS 轮询间隔（秒）
    sensor_stale_seconds: float = 1.5   # 传感器数据过期判定（秒）
    sensor_fail_limit: int = 10         # 传感器连续失败次数上限
    loop_wait: float = 0.01             # 监控循环步长（秒）
    lift_timeout_seconds: float = 20.0  # 抬升到位等待上限（秒）
    step_detect_kwargs: dict = field(default_factory=dict)


def _fmt_speed(value):
    value = float(value)
    return str(int(value)) if value.is_integer() else str(value)


def _event_to_dict(event):
    if event is None:
        return None
    if is_dataclass(event) and not isinstance(event, type):
        return asdict(event)
    if hasattr(event, '__dict__'):
        return dict(vars(event))
    return event


def run_drill_cycle(io, sensor, params=None, detector=None,
                    progress=None, progress_interval=0.5,
                    clock=time.monotonic, sleep=time.sleep, log=print):
    """执行一次完整钻孔循环，返回结果字典。

    期望 io 提供：send(line) / read_response(markers=None, timeout=...) /
    query_positions(timeout) / wait_z(target_z, timeout) / fault() / clear_fault()；
    期望 sensor 提供：latest() -> (monotonic, raw, value) | None、
    consecutive_failures / last_error / total_samples。

    progress(info) 可选：钻孔监控期间每 progress_interval 秒回调一次，
    info={'elapsed','z','depth','sample'}，供现场界面显示实时状态。
    """
    params = params or DrillParams()
    detector = detector or StepDetector(**params.step_detect_kwargs)
    started = clock()
    result = {
        'status': None, 'detail': None,
        'z_start': None, 'z_stop': None, 'depth_steps': None,
        'z_lift_target': None, 'lifted': False,
        'elapsed_seconds': None, 'drill_off_verified': None,
        'trigger': None, 'sensor_samples': None,
        'drilling_started': False,
    }

    def finish(status, detail=None, trigger=None):
        result['status'] = status
        result['detail'] = detail
        result['trigger'] = _event_to_dict(trigger)
        result['elapsed_seconds'] = clock() - started
        result['sensor_samples'] = getattr(sensor, 'total_samples', None)
        log(f"[DC] 结束: status={status}"
            + (f"，详情：{detail}" if detail else "")
            + f"（用时 {result['elapsed_seconds']:.1f}s）")
        return result

    # ---------- PRECHECK（未执行任何动作前的失败直接返回） ----------
    io.clear_fault()
    pos = io.query_positions(timeout=1.0)
    if pos is None:
        return finish('NO_POSITION', '读取 POS 失败，未执行任何动作')
    result['z_start'] = pos['Z']
    log(f"[DC] 开始：Z_start={result['z_start']}，进给={_fmt_speed(params.feed_speed)} 步/秒，"
        f"安全限深={params.safe_depth_steps} 步，抬升={params.lift_steps} 步")

    deadline = clock() + params.sensor_stale_seconds
    while sensor.latest() is None:
        if clock() > deadline:
            return finish('SENSOR_FAULT', '传感器无有效读数，未执行任何动作')
        sleep(params.loop_wait)

    # ---------- 启动钻孔 ----------
    io.send(f'SPEED {_fmt_speed(params.feed_speed)}')
    io.read_response(timeout=0.5)
    io.send('DRILL ON')
    ack = io.read_response(timeout=1.0)
    if 'DRILL ON' not in (ack or ''):
        return _stop_and_lift(io, sensor, params, result, finish, log, clock,
                              'DRILL_ON_FAILED', f'未确认钻头启动：{ack!r}')

    result['drilling_started'] = True
    io.send('ZDOWN')
    ack = io.read_response(timeout=1.0)
    if 'MOVE Z DOWN' not in (ack or ''):
        return _stop_and_lift(io, sensor, params, result, finish, log, clock,
                              'NO_ZDOWN', f'未确认开始下钻：{ack!r}')

    # ---------- 监控循环 ----------
    status = detail = trigger = None
    last_pos_poll = 0.0
    last_fresh = clock()
    last_progress = 0.0
    last_pos = None
    while True:
        now = clock()
        if now - started >= params.timeout_seconds:
            status, detail = 'TIMEOUT', f'超过 {params.timeout_seconds:.0f}s 未检测到阶跃'
            break
        fault = io.fault()
        if fault:
            status = 'LIMIT_STOP' if 'LIMIT_STOP' in fault else 'MOTION_FAULT'
            detail = fault
            break
        sample = sensor.latest()
        if sample is not None and now - sample[0] <= params.sensor_stale_seconds:
            last_fresh = now
            event = detector.update(sample[0], sample[2])
            if event is not None:
                status, detail, trigger = 'BREAKTHROUGH', '检测到阶跃（刺穿跌落）', event
                break
        if getattr(sensor, 'consecutive_failures', 0) >= params.sensor_fail_limit:
            status = 'SENSOR_FAULT'
            detail = (f'连续失败 {sensor.consecutive_failures} 次：'
                      f'{getattr(sensor, "last_error", None)}')
            break
        if now - last_fresh > params.sensor_stale_seconds:
            status = 'SENSOR_FAULT'
            detail = f'传感器数据超过 {params.sensor_stale_seconds:.1f}s 未更新'
            break
        if now - last_pos_poll >= params.pos_poll_interval:
            last_pos_poll = now
            pos = io.query_positions(timeout=0.6)
            if pos is not None:
                last_pos = pos
                depth = pos['Z'] - result['z_start']
                if depth >= params.safe_depth_steps:
                    status = 'DEPTH_LIMIT'
                    detail = f'下钻 {depth} 步 ≥ 安全限深 {params.safe_depth_steps} 步'
                    break
        if progress is not None and now - last_progress >= progress_interval:
            last_progress = now
            progress({
                'elapsed': now - started,
                'z': last_pos['Z'] if last_pos else None,
                'depth': (last_pos['Z'] - result['z_start']) if last_pos else None,
                'sample': sample,
            })
        sleep(params.loop_wait)

    return _stop_and_lift(io, sensor, params, result, finish, log, clock,
                          status, detail, trigger)


def _stop_and_lift(io, sensor, params, result, finish, log, clock,
                   status, detail=None, trigger=None):
    """统一中止/收尾路径：STOP -> 复核 -> 记录 -> 恢复速度 -> 抬升。"""
    # 1) 停：STOP 先关钻头，再立即清目标停 Z
    io.send('STOP')
    ack = io.read_response(markers=('STOP',), timeout=1.0)
    if not ack:
        io.read_response(timeout=0.5)

    # 2) 复核钻头已停
    io.send('DRILL STATUS')
    status_line = io.read_response(timeout=0.5)
    result['drill_off_verified'] = 'DRILL OFF' in (status_line or '')
    if not result['drill_off_verified']:
        log(f'[DC] 警告：DRILL STATUS 未确认停止（{status_line!r}），请人工检查钻头')

    # 3) 记录停止位置与钻深
    pos = io.query_positions(timeout=1.0)
    z_stop = pos['Z'] if pos else None
    result['z_stop'] = z_stop
    if z_stop is not None:
        result['depth_steps'] = z_stop - result['z_start']

    # 4) 恢复全轴速度（SPEED 是全轴设置，必须恢复）
    io.send(f'SPEED {_fmt_speed(params.normal_speed)}')
    io.read_response(timeout=0.3)

    # 5) 抬升固定距离
    if z_stop is not None:
        target = z_stop - int(params.lift_steps)
        result['z_lift_target'] = target
        io.send(f'Z -{int(params.lift_steps)}')
        io.read_response(timeout=0.3)
        result['lifted'] = bool(io.wait_z(target, timeout=params.lift_timeout_seconds))
        if not result['lifted']:
            log(f'[DC] 警告：抬升未确认到位（目标 Z={target}），请人工确认')
    else:
        log('[DC] 警告：读不到停止位置，无法自动抬升，请人工确认后手动抬升')

    log(f"[DC] 汇总：钻深={result['depth_steps']} 步，Z_stop={result['z_stop']}，"
        f"抬升到={result['z_lift_target']}（lifted={result['lifted']}），"
        f"钻头停止已复核={result['drill_off_verified']}")
    return finish(status, detail, trigger)
