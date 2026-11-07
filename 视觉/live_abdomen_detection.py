# -*- coding: utf-8 -*-
"""从 LRCP imx317 连拍或持续读取画面并检测腹部 P2。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from abdomen_spot_detector import detect_frame, detect_multiframe, imwrite_unicode, imread_unicode


LIVE_WINDOW_NAME = "LRCP full frame - P2 live detection"
VISION_CAPTURE_WINDOW_NAME = "VIS automatic detection - closes after capture"


def build_capture_preview(image, captured_count: int, count: int):
    """从原图绘制采集状态，避免叠加检测器自带的左上角文字。"""
    shown = image.copy()
    try:
        frame_result = detect_frame(image)
        points = [tuple(int(round(value)) for value in point)
                  for point in frame_result.points_image]
        color = (0, 255, 0)
        for point in points:
            cv2.circle(shown, point, 18, color, 6)
        cv2.line(shown, points[0], points[1], color, 6)
        cv2.putText(shown, "P2 target",
                    (min(point[0] for point in points), min(point[1] for point in points) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
        status = f"DETECTED  confidence={frame_result.confidence:.2f}"
    except ValueError as exc:
        status = f"NOT DETECTED: {str(exc)[:72]}"
        color = (0, 0, 255)
    lines = [
        f"VIS automatic capture {captured_count}/{count}",
        status,
        "Window closes automatically; Q / ESC / close = abort without XY move",
    ]
    for index, line in enumerate(lines):
        cv2.putText(shown, line, (40, 65 + index * 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.05, color, 3)
    return shown


def _load_directshow_dependencies():
    """优先使用已安装包；开发环境可回退到项目内临时依赖。"""
    try:
        from pygrabber.dshow_graph import FilterGraph
        from comtypes.client import PumpEvents
        return FilterGraph, PumpEvents
    except ImportError:
        fallback = Path(__file__).resolve().parents[1] / "camera_probe" / "deps"
        if fallback.exists():
            sys.path.insert(0, str(fallback))
            from pygrabber.dshow_graph import FilterGraph
            from comtypes.client import PumpEvents
            return FilterGraph, PumpEvents
        raise RuntimeError("缺少 pygrabber/comtypes，请运行: pip install pygrabber comtypes")


def capture_frames(
    camera_name: str = "LRCP imx317",
    count: int = 7,
    width: int = 3840,
    height: int = 2160,
    interval_seconds: float = 0.0,
    timeout_seconds: float | None = None,
    show_preview: bool = False,
    preview_width: int = 1280,
    preview_builder=None,
    preview_window_name: str = VISION_CAPTURE_WINDOW_NAME,
) -> list:
    """采集原始帧；可由调用方提供专用预览绘制函数和窗口标题。"""
    if preview_builder is None:
        preview_builder = build_capture_preview
    FilterGraph, PumpEvents = _load_directshow_dependencies()
    graph = FilterGraph()
    devices = graph.get_input_devices()
    if camera_name not in devices:
        raise RuntimeError(f"未找到相机 {camera_name}；当前设备: {devices}")
    graph.add_video_input_device(devices.index(camera_name))

    formats = graph.get_input_device().get_formats()
    matching = [f for f in formats if f["width"] == width and f["height"] == height]
    if not matching:
        raise RuntimeError(f"相机不支持 {width}x{height}")
    # 同分辨率优先 MJPG，避免 4K YUY2 只有很低帧率。
    matching.sort(key=lambda f: (f["media_type_str"] != "MJPG", f["index"]))
    graph.get_input_device().set_format(matching[0]["index"])

    frames = []

    def callback(image):
        frames.append(image.copy())

    graph.add_sample_grabber(callback)
    graph.add_null_render()
    graph.prepare_preview_graph()
    graph.run()
    preview_cancelled = False

    def show_capture_preview(image, captured_count: int) -> None:
        nonlocal preview_cancelled
        if not show_preview:
            return
        shown = preview_builder(image, captured_count, count)
        scale = min(1.0, max(320, preview_width) / shown.shape[1])
        display = cv2.resize(shown, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.imshow(preview_window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), ord("Q"), 27) or _window_was_closed(preview_window_name):
            preview_cancelled = True

    def wait_with_preview(seconds: float) -> None:
        nonlocal preview_cancelled
        deadline = time.monotonic() + max(0.0, seconds)
        while time.monotonic() < deadline and not preview_cancelled:
            if show_preview:
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), ord("Q"), 27) or _window_was_closed(preview_window_name):
                    preview_cancelled = True
                    break
            else:
                time.sleep(min(0.03, max(0.0, deadline - time.monotonic())))

    try:
        if show_preview:
            cv2.namedWindow(preview_window_name, cv2.WINDOW_NORMAL)
        time.sleep(1.0)
        attempts = 0
        deadline = time.monotonic() + (
            timeout_seconds if timeout_seconds is not None else max(10.0, count * 3.0)
        )
        last_sample_time = 0.0
        while len(frames) < count and attempts < count * 3 and time.monotonic() < deadline:
            if frames and interval_seconds > 0:
                wait = interval_seconds - (time.monotonic() - last_sample_time)
                if wait > 0:
                    wait_with_preview(min(wait, max(0.0, deadline - time.monotonic())))
            if preview_cancelled:
                break
            before = len(frames)
            graph.grab_frame()
            PumpEvents(min(0.7, max(0.0, deadline - time.monotonic())))
            if len(frames) > before:
                last_sample_time = time.monotonic()
                show_capture_preview(frames[-1], len(frames))
            attempts += 1
    finally:
        graph.stop()
        if show_preview:
            try:
                cv2.destroyWindow(preview_window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
    if preview_cancelled:
        raise RuntimeError("用户关闭了VIS检测窗口，本轮已取消，未发送XY移动")
    if len(frames) < count:
        raise RuntimeError(f"只采集到 {len(frames)}/{count} 帧")
    return frames[:count]


def _configure_graph(camera_name: str, width: int, height: int):
    """创建指定分辨率的 DirectShow 图；返回图对象和事件泵。"""
    FilterGraph, PumpEvents = _load_directshow_dependencies()
    graph = FilterGraph()
    devices = graph.get_input_devices()
    if camera_name not in devices:
        raise RuntimeError(f"未找到相机 {camera_name}；当前设备: {devices}")
    graph.add_video_input_device(devices.index(camera_name))
    formats = graph.get_input_device().get_formats()
    matching = [item for item in formats if item["width"] == width and item["height"] == height]
    if not matching:
        raise RuntimeError(f"相机不支持 {width}x{height}")
    matching.sort(key=lambda item: (item["media_type_str"] != "MJPG", item["index"]))
    graph.get_input_device().set_format(matching[0]["index"])
    return graph, PumpEvents


def _single_frame_payload(frame_result) -> dict:
    points = [[float(x), float(y)] for x, y in frame_result.points_image]
    center = [sum(point[axis] for point in points) / 2.0 for axis in (0, 1)]
    vector = [points[1][axis] - points[0][axis] for axis in (0, 1)]
    return {
        "target_name": "P2",
        "target_definition": "abdominal_penultimate_pair",
        "coordinate_system": "full_image_pixels",
        "points": points,
        "center": center,
        "angle_deg": round(float(cv2.fastAtan2(vector[1], vector[0])), 4),
        "confidence": round(float(frame_result.confidence), 4),
    }


def _load_xy_calibration() -> dict | None:
    path = Path(__file__).resolve().parents[1] / "pc" / "xy_pixel_calibration.json"
    if not path.exists():
        return None
    return json.loads(path.read_text(encoding="utf-8"))


def _filtered_payload(point_history, confidence_history, calibration, stable_samples, stable_jitter):
    if not point_history:
        return None
    points_stack = np.asarray(point_history, dtype=np.float64)
    points = np.median(points_stack, axis=0)
    centers = points_stack.mean(axis=1)
    center = points.mean(axis=0)
    center_distances = np.linalg.norm(centers - center, axis=1)
    jitter = float(np.median(center_distances))
    vector = points[1] - points[0]
    confidence = float(np.median(confidence_history))
    stable = len(point_history) >= stable_samples and jitter <= stable_jitter and confidence >= 0.55
    payload = {
        "target_name": "P2",
        "coordinate_system": "full_image_pixels",
        "filter": "rolling_median",
        "sample_count": len(point_history),
        "points": points.tolist(),
        "center": center.tolist(),
        "angle_deg": round(float(cv2.fastAtan2(float(vector[1]), float(vector[0]))), 4),
        "confidence": round(confidence, 4),
        "jitter_px": round(jitter, 4),
        "stable": stable,
    }
    if calibration is not None:
        reference = np.asarray(calibration["reference_p2_center_px"], dtype=np.float64)
        pixel_error = reference - center
        steps_per_pixel = np.asarray(calibration["steps_per_pixel"], dtype=np.float64)
        step_delta = steps_per_pixel @ pixel_error
        payload.update({
            "reference_center": reference.tolist(),
            "pixel_error": pixel_error.tolist(),
            "xy_correction_steps": step_delta.tolist(),
            "xy_correction_mm": (step_delta / (200.0 / 12.0)).tolist(),
        })
    return payload


def _draw_feedback(raw, payload, status: str):
    shown = raw.copy()
    color = (0, 255, 0) if status == "STABLE" else (0, 200, 255)
    if payload is not None:
        points = [tuple(int(round(value)) for value in point) for point in payload["points"]]
        center = tuple(int(round(value)) for value in payload["center"])
        cv2.line(shown, points[0], points[1], color, 6)
        for point in points:
            cv2.circle(shown, point, 18, color, 6)
        cv2.drawMarker(shown, center, color, cv2.MARKER_CROSS, 38, 5)
        lines = [
            f"STATUS: {status}   samples={payload['sample_count']}",
            f"P2 center: u={payload['center'][0]:.2f}  v={payload['center'][1]:.2f}",
            f"confidence={payload['confidence']:.3f}  jitter={payload['jitter_px']:.2f}px",
        ]
        if "xy_correction_steps" in payload:
            steps = payload["xy_correction_steps"]
            mm = payload["xy_correction_mm"]
            lines.append(f"XY correction: X={steps[0]:+.2f}  Y={steps[1]:+.2f} step")
            lines.append(f"XY correction: X={mm[0]:+.3f}  Y={mm[1]:+.3f} mm")
        for index, line in enumerate(lines):
            cv2.putText(shown, line, (40, 65 + index * 48),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.15, color, 3)
    else:
        cv2.putText(shown, f"STATUS: {status}", (40, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.15, (0, 0, 255), 3)
    return shown


def _save_live_result(output: Path, raw, frame_result, payload=None, annotated=None) -> None:
    output.mkdir(parents=True, exist_ok=True)
    imwrite_unicode(output / "latest_raw.jpg", raw)
    imwrite_unicode(output / "latest_annotated.jpg", annotated if annotated is not None else frame_result.annotated)
    imwrite_unicode(output / "latest_canonical.jpg", frame_result.canonical)
    imwrite_unicode(output / "latest_spot_mask.png", frame_result.spot_mask)
    (output / "latest_result.json").write_text(
        json.dumps(payload or _single_frame_payload(frame_result), ensure_ascii=False, indent=2),
        encoding="utf-8",
    )


def _window_was_closed(window_name: str) -> bool:
    """返回用户是否点了窗口关闭按钮；窗口销毁后部分 OpenCV 后端会抛异常。"""
    try:
        return cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def run_live(
    camera_name: str,
    output: Path,
    width: int = 3840,
    height: int = 2160,
    preview_width: int = 1280,
    detect_every: int = 1,
    max_frames: int = 0,
    show_window: bool = True,
    filter_window: int = 7,
    stable_samples: int = 5,
    stable_jitter: float = 2.0,
) -> int:
    """持续读取完整画面。关窗/Q/Esc 保存并退出，S 保存后继续。"""
    graph, PumpEvents = _configure_graph(camera_name, width, height)
    received = []

    def callback(image):
        received[:] = [image.copy()]

    graph.add_sample_grabber(callback)
    graph.add_null_render()
    graph.prepare_preview_graph()
    graph.run()
    last_raw = None
    last_valid_raw = None
    last_result = None
    last_payload = None
    last_annotated = None
    frame_index = 0
    missed = 0
    points_history = deque(maxlen=max(3, filter_window))
    confidence_history = deque(maxlen=max(3, filter_window))
    calibration = _load_xy_calibration()
    previous_status = None
    exit_reason = "采集结束"
    try:
        time.sleep(1.0)
        if show_window:
            cv2.namedWindow(LIVE_WINDOW_NAME, cv2.WINDOW_NORMAL)
        while max_frames <= 0 or frame_index < max_frames:
            received.clear()
            graph.grab_frame()
            PumpEvents(0.35)
            if not received:
                continue
            raw = received[-1]
            last_raw = raw
            frame_index += 1
            current_result = last_result
            if last_result is None or frame_index % max(1, detect_every) == 0:
                try:
                    current_result = detect_frame(raw)
                    last_result = current_result
                    last_valid_raw = raw
                    points_history.append(np.asarray(current_result.points_image, dtype=np.float64))
                    confidence_history.append(float(current_result.confidence))
                    missed = 0
                except ValueError as exc:
                    current_result = None
                    missed += 1

            last_payload = _filtered_payload(
                points_history, confidence_history, calibration, stable_samples, stable_jitter
            )
            if current_result is None and missed >= 3:
                status = "LOST"
            elif last_payload is None or last_payload["sample_count"] < stable_samples:
                status = "WARMING"
            elif last_payload["stable"]:
                status = "STABLE"
            else:
                status = "TRACKING"
            if status != previous_status:
                print(f"frame {frame_index}: {status}")
                previous_status = status

            shown = _draw_feedback(raw, last_payload, status)
            last_annotated = shown
            if show_window:
                scale = min(1.0, preview_width / shown.shape[1])
                display = cv2.resize(shown, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
                cv2.putText(display, "Close/Q/ESC: save & quit   S: save", (20, display.shape[0] - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
                cv2.imshow(LIVE_WINDOW_NAME, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    exit_reason = "按键退出"
                    break
                if key == ord("s") and last_result is not None:
                    _save_live_result(output, last_valid_raw, last_result, last_payload, last_annotated)
                    print(f"saved: {output.resolve()}")
                if _window_was_closed(LIVE_WINDOW_NAME):
                    exit_reason = "窗口已关闭"
                    break
    finally:
        graph.stop()
        if show_window:
            cv2.destroyAllWindows()
    if last_raw is None or last_result is None or last_valid_raw is None:
        if last_raw is not None:
            output.mkdir(parents=True, exist_ok=True)
            imwrite_unicode(output / "failed_latest_raw.jpg", last_raw)
        print(f"实时画面读取正常，但没有可信 P2；诊断原图已保存到 {output.resolve()}")
        return 2
    _save_live_result(output, last_valid_raw, last_result, last_payload, last_annotated)
    print(f"{exit_reason}；多帧融合结果已保存到: {output.resolve()}")
    print(json.dumps(last_payload, ensure_ascii=False, indent=2))
    return 0


def build_result_preview(record_path: Path):
    """从本轮存档绘制融合点及实际选中的工具目标，不访问相机。"""
    record = json.loads(record_path.read_text(encoding='utf-8'))
    shown = imread_unicode(record_path.parent / 'raw_00.jpg')
    detection = record['detection']
    calculation = record['calculation']
    for point in detection['points']:
        center = tuple(int(round(value)) for value in point)
        cv2.circle(shown, center, 18, (0, 255, 0), 5)
    selected = tuple(int(round(value)) for value in calculation['selected_point_px'])
    cv2.drawMarker(shown, selected, (0, 0, 255), cv2.MARKER_CROSS, 65, 6)
    target = calculation['target_xy_steps']
    lines = [
        f"VIS result: {record_path.parent.name} (saved frame, not live)",
        f"{detection.get('target_name', 'P2')} score={detection['confidence']:.3f}  stable={detection['stable_frames']}/7",
        f"RED = selected target; XY command: X={target['X']} Y={target['Y']} steps",
        "Preview only; automatic XY continues. Close / Q / Esc closes this image.",
    ]
    for index, line in enumerate(lines):
        cv2.putText(shown, line, (40, 65 + index * 55),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
    return shown


def show_saved_result(record_path: Path, preview_width: int = 1280) -> int:
    shown = build_result_preview(record_path)
    scale = min(1.0, max(1, preview_width) / shown.shape[1])
    display = cv2.resize(shown, None, fx=scale, fy=scale)
    window = 'VIS result - ' + record_path.parent.name
    try:
        cv2.imshow(window, display)
        while True:
            if cv2.waitKey(30) & 0xFF in (27, ord('q'), ord('Q')):
                break
            if _window_was_closed(window):
                break
    finally:
        cv2.destroyAllWindows()
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="LRCP 实时检测蟑螂腹部 P2")
    parser.add_argument("--frames", type=int, default=7, help="连拍帧数，默认 7")
    parser.add_argument("--result-json", type=Path, help="显示VIS存档结果，不打开相机")
    parser.add_argument("--output-dir", default="vision_output/live", help="输出目录")
    parser.add_argument("--camera", default="LRCP imx317", help="DirectShow 相机名称")
    parser.add_argument("--live", action="store_true", help="持续显示完整画面并逐帧检测")
    parser.add_argument("--detect-every", type=int, default=1, help="实时模式每隔多少帧检测一次")
    parser.add_argument("--preview-width", type=int, default=1280, help="预览窗口宽度；识别仍使用4K原图")
    parser.add_argument("--filter-window", type=int, default=7, help="实时坐标滚动中位数窗口")
    parser.add_argument("--stable-samples", type=int, default=5, help="判定稳定所需有效样本数")
    parser.add_argument("--stable-jitter", type=float, default=2.0, help="稳定状态最大中位抖动像素")
    parser.add_argument("--max-frames", type=int, default=0, help=argparse.SUPPRESS)
    parser.add_argument("--no-window", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()

    if args.result_json is not None:
        return show_saved_result(args.result_json, args.preview_width)

    output = Path(args.output_dir)
    if args.live:
        return run_live(
            camera_name=args.camera,
            output=output,
            preview_width=args.preview_width,
            detect_every=args.detect_every,
            max_frames=args.max_frames,
            show_window=not args.no_window,
            filter_window=args.filter_window,
            stable_samples=args.stable_samples,
            stable_jitter=args.stable_jitter,
        )

    output.mkdir(parents=True, exist_ok=True)
    frames = capture_frames(camera_name=args.camera, count=args.frames)
    for index, image in enumerate(frames):
        imwrite_unicode(output / f"raw_{index:02d}.jpg", image)
    result = detect_multiframe(frames)
    for index, (image, frame_result) in enumerate(zip(frames, result["frame_results"])):
        imwrite_unicode(output / f"annotated_{index:02d}.jpg", frame_result.annotated)
        if index == 0:
            imwrite_unicode(output / "canonical.jpg", frame_result.canonical)
            imwrite_unicode(output / "spot_mask.png", frame_result.spot_mask)

    payload = {key: value for key, value in result.items() if key != "frame_results"}
    (output / "result.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["confidence"] >= 0.55 else 2


if __name__ == "__main__":
    raise SystemExit(main())
