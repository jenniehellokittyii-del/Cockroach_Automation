import json
import subprocess
import sys
import time
from datetime import datetime
import serial_motion as motion
from serial_motion import (goto_vision_position, goto_vision_position_without_home,
                           query_positions, move_continuous)
from settings import *
from vis_head import capture_and_detect_head

vis_head_fast_ready = False

def calculate_vision_target(
    payload,
    calibration,
    tool_calibration,
    point_index=None,
    point_compensation=None,
):
    """把用户选择的识别点换算为带工具偏移的绝对 XY 目标。"""
    vision_dir = PROJECT_ROOT / 'code' / 'vision'
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
    if motion.motion_fault:
        raise ValueError(f'运动故障，拒绝移动：{motion.motion_fault}')
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
        if choice.startswith('p'):
            choice = choice[1:]
        if choice.isdigit():
            index = int(choice) - 1
            if index in allowed_indices:
                return index
        print('输入无效，请输入列表中的点位编号，或输入 Q 结束。')

def capture_and_calculate_vision_target():
    """采集7帧、识别并保存结果，常驻显示结果并返回绝对XY目标。"""
    vision_dir = PROJECT_ROOT / 'code' / 'vision'
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

def run_automatic_vision(ser, skip_home=False, single_point=False, retry_capture=None):
    """进入视觉位、稳定识别并移动到针轴 XY；不换头、不动目标 Z。"""
    position_ready = (
        goto_vision_position_without_home(ser)
        if skip_home else goto_vision_position(ser)
    )
    if not position_ready:
        return False
    while True:
        try:
            session = capture_and_calculate_vision_target()
            target = session['first_calculation']['target_xy_steps']
            break
        except Exception as exc:
            print(f'VISION: 识别或换算失败，停在视觉位，未发送目标移动：{exc}')
            if retry_capture is None:
                return False
            if not retry_capture():
                print('VISION: 已取消重新识别。')
                return False
            # 只重试采图；不得把运动失败当成识别失败重新执行。
            pos = query_positions(ser)
            if (motion.motion_fault or not motion.vision_reference_ready
                    or pos is None or pos.get('X2') != pos.get('X')
                    or any(pos[axis] != VISION_POS_STEPS[axis] for axis in ('X', 'Y', 'Z'))):
                print(f'VISION: 位置或运动状态异常，禁止重新识别：{pos}；{motion.motion_fault}')
                return False
            print('VISION: 保持当前视觉位，重新采图识别（不回零、不重复取料）。')

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
    while remaining and not single_point:
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

def run_visual_head(ser, skip_home=False, single_point=False, retry_capture=None):
    """进入视觉位，识别头部四点，并按用户选择逐点移动。"""
    global vis_head_fast_ready
    # 跳过归零只要求本次机械零点已确认，由定位函数检查；
    # 自动化入口在取料前已经 HOME，不必先运行一次带 HOME 的头部识别。
    position_ready = (
        goto_vision_position_without_home(ser)
        if skip_home else goto_vision_position(ser)
    )
    if not position_ready:
        return False

    while True:
        try:
            points = capture_and_detect_head()
            if len(points) != 4:
                raise ValueError(f'应识别到 4 个头部点，实际得到 {len(points)} 个')
            calibration = json.loads(VISION_PIXEL_CALIBRATION.read_text(encoding='utf-8'))
            tool_calibration = json.loads(VISION_TOOL_CALIBRATION.read_text(encoding='utf-8'))
            head_point_compensation = json.loads(
                VISION_HEAD_POINT_COMPENSATION.read_text(encoding='utf-8')
            )
            break
        except Exception as exc:
            print(f'VIS_HEAD: 头部识别失败，停在视觉位：{exc}')
            if retry_capture is None or not retry_capture():
                return False
            pos = query_positions(ser)
            if (motion.motion_fault or not motion.vision_reference_ready
                    or pos is None or pos.get('X2') != pos.get('X')
                    or any(pos[axis] != VISION_POS_STEPS[axis] for axis in ('X', 'Y', 'Z'))):
                print(f'VIS_HEAD: 位置或运动状态异常，禁止重试：{pos}；{motion.motion_fault}')
                return False
            print('VIS_HEAD: 保持视觉位，重新采图进行头部识别。')

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
        if single_point:
            break

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
