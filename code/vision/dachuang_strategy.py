"""将大创六点策略接入运动工程；所有输出坐标均为采集原图像素。"""
from functools import lru_cache
from types import SimpleNamespace

import cv2
import numpy as np

from dachuang.six_point_locator import MODEL
from dachuang.camera_inference import infer

MODE = 'geometry'  # 新六点沿用轮廓相对几何约束，不使用旧纹理点位
MAX_SIDE = 960
STABLE_THRESHOLD_PX = 8.0


@lru_cache(maxsize=1)
def load_model(mode='geometry'):
    model_path = MODEL.with_name('six_point_geometry_20261106.npz') if mode == 'geometry' else MODEL
    with np.load(model_path, allow_pickle=False) as archive:
        return {key: archive[key].copy() for key in archive.files}


def detect_six_points(image, mode=None):
    mode = MODE if mode is None else mode
    if mode not in ('auto', 'texture', 'geometry'):
        raise ValueError(f'未知六点识别模式：{mode}')
    height, width = image.shape[:2]
    scale = min(1.0, MAX_SIDE / max(height, width))
    small = cv2.resize(image, (round(width * scale), round(height * scale)))
    model = load_model(mode)
    result = infer(small, model, mode)
    if result['points'] is None:
        raise ValueError(result['error'] or '大创策略未检测到六点')
    points = result['points']
    info = {'method': result['method'], 'segmentation_error': result['segmentation_error']}
    points = np.array(points, dtype=float, copy=True)
    if points.shape != (6, 2):
        raise ValueError('六点坐标无效：应返回6组像素坐标')
    points *= [width / small.shape[1], height / small.shape[0]]
    if (points.shape != (6, 2) or not np.isfinite(points).all()
            or np.any(points < 0) or np.any(points[:, 0] >= width)
            or np.any(points[:, 1] >= height)):
        raise ValueError('六点坐标无效或超出原图范围')
    return points, info


def detect_head_points(image, split_ratio=None):
    # split_ratio 仅兼容旧调用，六点策略不使用固定分区比例。
    points, info = detect_six_points(image)
    return points[:4].tolist(), info


def build_capture_preview(image, captured_count, count):
    shown = image.copy()
    try:
        points, info = detect_six_points(image)
        for index, point in enumerate(points[4:], 1):
            xy = tuple(np.rint(point).astype(int))
            cv2.circle(shown, xy, 12, (0, 0, 255), 3)
            cv2.putText(shown, str(index), (xy[0] + 15, xy[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        status = info['method']
    except (ValueError, cv2.error) as exc:
        status = f'NO DETECTION: {str(exc)[:80]}'
    cv2.putText(shown, f'{captured_count}/{count} {status}', (30, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
    return shown


def detect_multiframe(images):
    results, errors = [], []
    for image in images:
        try:
            points, info = detect_six_points(image)
            annotated = image.copy()
            for index, point in enumerate(points[4:], 1):
                xy = tuple(np.rint(point).astype(int))
                cv2.circle(annotated, xy, 12, (0, 0, 255), 3)
                cv2.putText(annotated, str(index), xy, cv2.FONT_HERSHEY_SIMPLEX,
                            1, (0, 255, 255), 2)
            cv2.putText(annotated, info['method'], (30, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 165, 255), 2)
            results.append(SimpleNamespace(points_image=points[4:],
                                           annotated=annotated, info=info))
        except (ValueError, cv2.error) as exc:
            errors.append(str(exc))
    if not results:
        raise ValueError('所有帧六点识别失败：' + '; '.join(errors))
    stack = np.asarray([r.points_image for r in results])
    median = np.median(stack, axis=0)
    deviations = np.linalg.norm(stack - median, axis=2).max(axis=1)
    stable = deviations <= STABLE_THRESHOLD_PX
    if stable.any():
        median = np.median(stack[stable], axis=0)
    vector = median[1] - median[0]
    methods = [r.info['method'] for r in results]
    if any('ESTIMATE' in method for method in methods):
        print('大创策略：存在轮廓几何估计点，未识别特定斑纹；请核对预览。')
    return {
        'target_name': 'dachuang_points_5_6',
        'target_definition': 'reference_model_abdomen_pair',
        'coordinate_system': 'full_image_pixels',
        'points': median.tolist(), 'center': median.mean(axis=0).tolist(),
        'angle_deg': float(np.degrees(np.arctan2(vector[1], vector[0])) % 360),
        'confidence': float(np.exp(-np.median(deviations) / 12)),
        'confidence_definition': 'temporal_stability_only_not_anatomical_accuracy',
        'source_point_numbers': [5, 6], 'methods': methods,
        'successful_frames': len(results), 'failed_frames': len(errors),
        'stable_frames': int(stable.sum()), 'frame_results': results, 'errors': errors,
    }
