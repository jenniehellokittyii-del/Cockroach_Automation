# -*- coding: utf-8 -*-
"""VIS_HEAD 头部识别模块。

独立代码文件用途：
    1. 调用相机采集多帧
    2. 调用视觉/head.py 检测头部四点
    3. 对成功帧取中位数
    4. 在窗口显示头部四点，仅返回像素坐标，不保存文件

不负责机械回零和视觉位移动，这些仍由 motion_client.py 控制。
"""
import math
import sys
import time
from pathlib import Path

import cv2
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]

# ---------------- VIS_HEAD 可调参数 ----------------
CAMERA_NAME = 'LRCP imx317'
FRAME_COUNT = 7
SAMPLE_INTERVAL_SECONDS = 2.0
CAPTURE_TIMEOUT_SECONDS = 30.0
SETTLE_SECONDS = 2.0
MIN_SUCCESSFUL_FRAMES = 5
SPLIT_RATIO = 0.62
SHOW_CAPTURE_PREVIEW = True
HEAD_CAPTURE_WINDOW_NAME = 'VIS_HEAD - head P1 P2 P3 P4 capture'
HEAD_RESULT_WINDOW_NAME = 'VIS_HEAD - head pixel points (Q / ESC / close to continue)'


def _add_vision_path():
    """让本模块能够导入项目中的视觉识别代码。"""
    vision_dir = PROJECT_ROOT / '视觉'
    if str(vision_dir) not in sys.path:
        sys.path.insert(0, str(vision_dir))
    return vision_dir


def _draw_head_preview(image, points):
    """点旁只显示编号，完整原图坐标列在右侧面板，避免文字重叠。"""
    shown = image.copy()
    height, width = shown.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = max(0.55, height / 1080 * 0.8)
    thickness = max(1, int(round(height / 1080)))
    panel_width = max(340, int(round(height * 0.36)))
    panel = np.full((height, panel_width, 3), (24, 24, 24), dtype=np.uint8)
    margin = max(15, int(round(height * 0.02)))
    row_height = max(36, int(round(height * 0.055)))
    cv2.putText(panel, 'HEAD PIXELS (original)', (margin, row_height),
                font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    for index, (x, y) in enumerate(points, start=1):
        row_y = row_height * (index + 2)
        if not np.isfinite([x, y]).all():
            cv2.putText(panel, f'P{index}: not detected', (margin, row_y),
                        font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
            continue
        x, y = int(round(x)), int(round(y))
        cv2.circle(shown, (x, y), 12, (0, 0, 255), 3)
        cv2.circle(shown, (x, y), 5, (0, 0, 255), -1)
        label = f'P{index}'
        (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
        # P1/P2 向左标注，P3/P4 向右；上侧点向上、下侧点向下。
        label_x = x - text_width - 20 if index <= 2 else x + 20
        label_y = y - 20 if index in (1, 3) else y + text_height + 20
        label_x = max(0, min(width - text_width - 1, label_x))
        label_y = max(text_height, min(height - baseline - 1, label_y))
        cv2.putText(shown, label, (label_x, label_y),
                    font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
        cv2.putText(panel, f'P{index}: ({x}, {y})', (margin, row_y),
                    font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
    return np.concatenate((shown, panel), axis=1)


def build_head_capture_preview(image, captured_count, count, split_ratio=SPLIT_RATIO):
    """采图时调用头部算法，避免使用腹部 P2 预览。"""
    from head import detect_head_points

    try:
        points, _ = detect_head_points(image, split_ratio=split_ratio)
        shown = _draw_head_preview(image, points)
        status = 'HEAD DETECTED' if np.isfinite(points).all() else 'HEAD POINTS INCOMPLETE'
    except Exception as exc:
        shown = _draw_head_preview(image, [(np.nan, np.nan)] * 4)
        status = f'HEAD NOT DETECTED: {str(exc)[:72]}'
    for index, line in enumerate((f'VIS_HEAD capture {captured_count}/{count}', status,
                                  'Q / ESC / close = cancel')):
        cv2.putText(shown, line, (40, 65 + index * 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, (0, 0, 255), 3)
    return shown


def _show_head_result(image, points):
    """保留四点中位数预览，直到用户按 Q/ESC 或关闭窗口。"""
    shown = _draw_head_preview(image, points)
    scale = min(1.0, 1280 / shown.shape[1])
    display = cv2.resize(shown, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    cv2.namedWindow(HEAD_RESULT_WINDOW_NAME, cv2.WINDOW_NORMAL)
    try:
        cv2.imshow(HEAD_RESULT_WINDOW_NAME, display)
        while True:
            key = cv2.waitKey(30) & 0xFF
            if key in (ord('q'), ord('Q'), 27):
                break
            if cv2.getWindowProperty(HEAD_RESULT_WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1:
                break
    finally:
        try:
            cv2.destroyWindow(HEAD_RESULT_WINDOW_NAME)
            cv2.waitKey(1)
        except cv2.error:
            pass


def capture_and_detect_head(
    camera_name=CAMERA_NAME,
    frame_count=FRAME_COUNT,
    sample_interval_seconds=SAMPLE_INTERVAL_SECONDS,
    capture_timeout_seconds=CAPTURE_TIMEOUT_SECONDS,
    settle_seconds=SETTLE_SECONDS,
    min_successful_frames=MIN_SUCCESSFUL_FRAMES,
    split_ratio=SPLIT_RATIO,
    show_capture_preview=SHOW_CAPTURE_PREVIEW,
):
    """
    采集多帧并识别头部四点。

    返回:
        median_points: [(x1, y1), (x2, y2), (x3, y3), (x4, y4)]
    """
    _add_vision_path()

    from head import detect_head_points
    from live_abdomen_detection import capture_frames

    print(f'VIS_HEAD: 机构稳定 {settle_seconds:.0f}s 后开始头部识别')
    time.sleep(settle_seconds)

    frames = capture_frames(
        camera_name=camera_name,
        count=frame_count,
        interval_seconds=sample_interval_seconds,
        timeout_seconds=capture_timeout_seconds,
        show_preview=show_capture_preview,
        preview_width=1280,
        preview_builder=lambda image, captured, count: build_head_capture_preview(
            image, captured, count, split_ratio=split_ratio,
        ),
        preview_window_name=HEAD_CAPTURE_WINDOW_NAME,
    )

    valid_results = []
    for index, frame in enumerate(frames):
        try:
            points, _ = detect_head_points(
                frame,
                split_ratio=split_ratio,
            )
            if any(
                math.isnan(float(point[0])) or math.isnan(float(point[1]))
                for point in points
            ):
                print(f'VIS_HEAD: 第 {index + 1} 帧四点不完整，已跳过')
                continue
            valid_results.append((index, points))
        except Exception as exc:
            print(f'VIS_HEAD: 第 {index + 1} 帧识别失败，已跳过：{exc}')

    if len(valid_results) < min_successful_frames:
        raise ValueError(
            f'仅 {len(valid_results)}/{frame_count} 帧识别成功，'
            f'至少需要 {min_successful_frames} 帧'
        )

    points_array = np.asarray(
        [points for _, points in valid_results],
        dtype=np.float64,
    )
    median_points_array = np.median(points_array, axis=0)
    median_points = [
        (int(round(float(point[0]))), int(round(float(point[1]))))
        for point in median_points_array
    ]

    print(f'VIS_HEAD: {len(valid_results)}/{frame_count} 帧识别成功')
    for index, (x, y) in enumerate(median_points, start=1):
        print(f'P{index}: ({x}, {y})')
    if show_capture_preview:
        first_index = valid_results[0][0]
        _show_head_result(frames[first_index], median_points)
    return median_points


if __name__ == '__main__':
    capture_and_detect_head()
