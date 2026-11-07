# -*- coding: utf-8 -*-
"""磁吸盖下方窗口内的蟑螂腹部倒数第二对黑点（P2）检测。

机构先移动到固定视觉位。算法在画面中央下半部寻找暴露的棕黄色腹部，
以腹部边界建立固定尺寸坐标，再进行局部背景归一化、左右有序匹配，
验证腹节序列和末对收窄后选择从尾端向上第二对黑点。
支持多帧中位数与几何/稳定性评分；该评分不是解剖目标命中概率。
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


CANONICAL_SIZE = (500, 900)  # width, height


def imread_unicode(path: str | Path) -> np.ndarray:
    data = np.fromfile(str(path), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"无法读取图片: {path}")
    return image


def imwrite_unicode(path: str | Path, image: np.ndarray) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    suffix = path.suffix or ".jpg"
    ok, encoded = cv2.imencode(suffix, image)
    if not ok:
        raise ValueError(f"无法编码图片: {path}")
    encoded.tofile(str(path))


@dataclass
class Candidate:
    x: float
    y: float
    area: float
    response: float
    side: str


@dataclass
class Pair:
    left: Candidate
    right: Candidate
    score: float

    @property
    def avg_y(self) -> float:
        return (self.left.y + self.right.y) / 2.0


@dataclass
class FrameResult:
    points_image: list[tuple[float, float]]
    points_canonical: list[tuple[float, float]]
    pairs: list[Pair]
    confidence: float
    tray_bbox: tuple[int, int, int, int]
    canonical: np.ndarray
    spot_mask: np.ndarray
    annotated: np.ndarray


def locate_abdomen_window(
    image: np.ndarray,
) -> tuple[np.ndarray, tuple[int, int], tuple[int, int, int, int]]:
    """在固定视觉位中央区域定位棕黄色腹部，并返回带余量的 ROI。"""
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 盖子和平台可能同为蓝色且会因曝光接近白色，因此不用夹具颜色定位。
    # 夹具可能旋转 90°；扩大到中央区域，但仍排除画面四周的机架和工具。
    search = np.zeros((height, width), np.uint8)
    cv2.rectangle(
        search,
        (int(width * 0.24), int(height * 0.24)),
        (int(width * 0.76), int(height * 0.92)),
        255,
        -1,
    )
    abdomen = cv2.inRange(hsv, np.array([3, 55, 35]), np.array([42, 255, 255]))
    abdomen = cv2.bitwise_and(abdomen, search)
    abdomen = cv2.morphologyEx(
        abdomen, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9)),
    )
    abdomen = cv2.morphologyEx(
        abdomen, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)),
    )
    contours, _ = cv2.findContours(abdomen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("中央识别区内未找到棕黄色腹部")

    image_area = image.shape[0] * image.shape[1]
    contours = [c for c in contours if cv2.contourArea(c) > image_area * 0.002]
    if not contours:
        raise ValueError("中央识别区内腹部区域过小，无法可靠定位")
    contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(contour)
    if not 0.65 <= w / max(h, 1) <= 2.0:
        raise ValueError(f"腹部外形比例异常: {w}x{h}")
    pad_x = max(18, int(w * 0.08))
    pad_y = max(18, int(h * 0.06))
    x0, y0 = max(0, x - pad_x), max(0, y - pad_y)
    x1 = min(width, x + w + pad_x)
    y1 = min(height, y + h + pad_y)
    return image[y0:y1, x0:x1].copy(), (x0, y0), (x0, y0, x1 - x0, y1 - y0)


def segment_roach(tray_roi: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """在下窗口 ROI 内按腹部颜色分割暴露主体。"""
    h, w = tray_roi.shape[:2]
    hsv = cv2.cvtColor(tray_roi, cv2.COLOR_BGR2HSV)
    candidate = cv2.inRange(hsv, np.array([3, 48, 30]), np.array([45, 255, 255]))
    candidate = cv2.morphologyEx(
        candidate, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    candidate = cv2.morphologyEx(
        candidate, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (41, 41)),
    )

    contours, _ = cv2.findContours(candidate, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("载具 ROI 内未找到蟑螂主体")
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < h * w * 0.25:
        raise ValueError("蟑螂主体面积过小，分割置信度不足")
    mask = np.zeros((h, w), np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, -1)
    return mask, contour


def rectify_roach(
    tray_roi: np.ndarray, mask: np.ndarray
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """按固定新夹具方向（头左、尾右）把暴露腹部归一化到 500x900。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("腹部掩膜为空")
    x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
    margin_x = max(3.0, w * 0.025)
    margin_y = max(3.0, h * 0.025)
    x0, x1 = max(0.0, x - margin_x), min(float(mask.shape[1] - 1), x + w + margin_x)
    y0, y1 = max(0.0, y - margin_y), min(float(mask.shape[0] - 1), y + h + margin_y)
    # 当前及后续装夹固定为头左、尾右。映射后保持头在标准图上方、尾在下方；
    # 画面上侧对应标准图左侧，因此工具目标索引仍指向同一解剖侧。
    source = np.array(
        [[x0, y0], [x0, y1], [x1, y1], [x1, y0]], dtype=np.float32
    )
    width, height = CANONICAL_SIZE
    destination = np.array(
        [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]],
        dtype=np.float32,
    )
    matrix = cv2.getPerspectiveTransform(source, destination)
    canonical = cv2.warpPerspective(tray_roi, matrix, (width, height))
    canonical_mask = cv2.warpPerspective(mask, matrix, (width, height))
    return canonical, canonical_mask, matrix


def _detect_candidates_at_contrast(
    relative_darkness: np.ndarray, body_mask: np.ndarray, contrast_threshold: float,
) -> tuple[list[Candidate], np.ndarray]:
    """在局部背景归一化后的相对暗度图上提取侧黑点。"""
    width, height = CANONICAL_SIZE

    valid = np.zeros_like(body_mask)
    # 目标位于浅色腹部；排除深色胸部与最尾端毛刺。
    valid[int(height * 0.06):int(height * 0.94), :] = 255
    inner_body = cv2.erode(body_mask, np.ones((9, 9), np.uint8))
    valid = cv2.bitwise_and(valid, inner_body)
    if not np.any(valid):
        raise ValueError("腹部有效区域为空")
    binary = np.uint8(relative_darkness > contrast_threshold) * 255
    binary = cv2.bitwise_and(binary, valid)
    binary = cv2.morphologyEx(
        binary, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    binary = cv2.morphologyEx(
        binary, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )

    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates: list[Candidate] = []
    for contour in contours:
        area = float(cv2.contourArea(contour))
        # 在固定500x900标准图中，真实侧黑点明显大于腹节细纹和尾端碎斑。
        # 保留收窄尾部的小黑点，同时排除原先18像素起的纹理候选。
        if not 240 <= area <= 2400:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if w < 4 or h < 4 or w > 80 or h > 100:
            continue
        aspect = w / max(h, 1)
        if not 0.30 <= aspect <= 3.0:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        hull_area = cv2.contourArea(cv2.convexHull(contour))
        solidity = area / hull_area if hull_area > 0 else 0
        if circularity < 0.22 or solidity < 0.52:
            continue
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        cx = float(moments["m10"] / moments["m00"])
        cy = float(moments["m01"] / moments["m00"])
        if width * 0.40 <= cx <= width * 0.60:
            continue  # 中央轴向高光/腹节缝，不是左右侧黑点
        contour_mask = np.zeros_like(binary)
        cv2.drawContours(contour_mask, [contour], -1, 255, -1)
        mean_response = float(cv2.mean(relative_darkness * 120, mask=contour_mask)[0])
        candidates.append(Candidate(
            x=cx, y=cy, area=area, response=mean_response,
            side="left" if cx < width / 2 else "right",
        ))
    return candidates, binary


def detect_candidates(canonical: np.ndarray, body_mask: np.ndarray) -> tuple[list[Candidate], np.ndarray]:
    """按局部背景相对暗度检测，减少全局曝光和局部阴影的影响。"""
    gray = cv2.cvtColor(canonical, cv2.COLOR_BGR2GRAY)
    # 闭运算填平小暗斑，估计其周围背景；黑帽响应除以背景后，
    # 统一增减曝光不会直接改变相对暗度。不能恢复过曝丢失的细节。
    background = cv2.morphologyEx(
        gray, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (61, 61)),
    )
    relative_darkness = ((background.astype(np.float32) - gray)
                         / np.maximum(background, 1))
    candidates: list[Candidate] = []
    combined_mask = np.zeros_like(body_mask)
    for contrast_threshold in (0.60, 0.55, 0.50, 0.45):
        found, binary = _detect_candidates_at_contrast(
            relative_darkness, body_mask, contrast_threshold,
        )
        combined_mask = cv2.bitwise_or(combined_mask, binary)
        for candidate in sorted(found, key=lambda item: -item.area):
            if any(np.hypot(candidate.x - old.x, candidate.y - old.y) < 45
                   for old in candidates):
                continue
            candidates.append(candidate)
    return candidates, combined_mask


def _pair_cost(left: Candidate, right: Candidate) -> float | None:
    width, height = CANONICAL_SIZE
    separation = right.x - left.x
    dy = abs(left.y - right.y)
    symmetry = abs((left.x + right.x) / 2 - width / 2)
    # 暴露腹部向尾端明显收窄，最末一对间距约为标准宽度的 0.28；
    # 必须保留该点对，才能按解剖顺序选择真正的倒数第二对。
    if not width * 0.20 <= separation <= width * 0.88:
        return None
    if dy > height * 0.065 or symmetry > width * 0.17:
        return None
    response_bonus = min((left.response + right.response) / 120.0, 1.0)
    return dy / (height * 0.065) + symmetry / (width * 0.17) - 0.35 * response_bonus


def pair_candidates(candidates: Iterable[Candidate]) -> list[Pair]:
    """用保持头尾顺序的动态规划全局匹配左右点，避免贪心错配。"""
    left = sorted((c for c in candidates if c.side == "left"), key=lambda c: c.y)
    right = sorted((c for c in candidates if c.side == "right"), key=lambda c: c.y)
    n, m = len(left), len(right)
    dp = np.full((n + 1, m + 1), -1e9, np.float64)
    parent: list[list[tuple[int, int, bool] | None]] = [[None] * (m + 1) for _ in range(n + 1)]
    dp[0, 0] = 0.0
    skip_penalty = 0.22
    for i in range(n + 1):
        for j in range(m + 1):
            if dp[i, j] < -1e8:
                continue
            if i < n and dp[i, j] - skip_penalty > dp[i + 1, j]:
                dp[i + 1, j] = dp[i, j] - skip_penalty
                parent[i + 1][j] = (i, j, False)
            if j < m and dp[i, j] - skip_penalty > dp[i, j + 1]:
                dp[i, j + 1] = dp[i, j] - skip_penalty
                parent[i][j + 1] = (i, j, False)
            if i < n and j < m:
                cost = _pair_cost(left[i], right[j])
                if cost is not None:
                    value = dp[i, j] + 2.0 - cost
                    if value > dp[i + 1, j + 1]:
                        dp[i + 1, j + 1] = value
                        parent[i + 1][j + 1] = (i, j, True)

    pairs: list[Pair] = []
    i, j = n, m
    while i > 0 or j > 0:
        state = parent[i][j]
        if state is None:
            break
        pi, pj, matched = state
        if matched:
            cost = _pair_cost(left[pi], right[pj])
            assert cost is not None
            pairs.append(Pair(left[pi], right[pj], score=float(np.clip(1.0 - cost / 2.0, 0, 1))))
        i, j = pi, pj
    pairs.sort(key=lambda pair: pair.avg_y)
    return pairs


def validated_abdominal_sequence(pairs: Iterable[Pair], tail_end_y: float | None = None) -> list[Pair]:
    """用腹节间距和末对收窄验证序列；缺少尾端依据时拒绝重新编号。

    参数适用于当前500x900、头上尾下的标准图与夹具；不是通用解剖分类器。
    """
    ordered = sorted(pairs, key=lambda pair: pair.avg_y)
    if len(ordered) < 3:
        raise ValueError(f"黑点序列不足3对，无法验证尾端和P2：当前{len(ordered)}对")
    height = CANONICAL_SIZE[1]
    chains: list[list[Pair]] = []
    verified: list[list[Pair]] = []
    for index, pair in enumerate(ordered):
        separation = pair.right.x - pair.left.x
        options = [chains[previous] + [pair] for previous in range(index)
                   if height * 0.105 <= pair.avg_y - ordered[previous].avg_y <= height * 0.228
                   and separation <= (ordered[previous].right.x - ordered[previous].left.x) * 1.15]
        chain = max(options, key=lambda value: (len(value), sum(item.score for item in value)),
                    default=[pair])
        chains.append(chain)
        if len(chain) < 3:
            continue
        previous_width = chain[-2].right.x - chain[-2].left.x
        if separation > previous_width * 0.70:
            continue
        if tail_end_y is not None:
            last_gap = pair.avg_y - chain[-2].avg_y
            if not 0 <= tail_end_y - pair.avg_y <= last_gap * 2.0:
                continue
        verified.append(chain)
    if not verified:
        raise ValueError("尾端点对不完整或腹节序列不可信，拒绝将前方点对当作P2")
    best = max(verified, key=lambda value: (len(value), value[-1].avg_y))
    for alternative in verified:
        if len(alternative) == len(best) and abs(alternative[-2].avg_y - best[-2].avg_y) > 45:
            raise ValueError("存在不同P2的竞争序列，拒绝输出不确定目标")
    return best


def select_p2(pairs: Iterable[Pair]) -> Pair:
    """只从已验证尾端收窄的腹节点对序列中取倒数第二对。"""
    return validated_abdominal_sequence(pairs)[-2]


def canonical_to_image(
    point: tuple[float, float], matrix: np.ndarray, offset: tuple[int, int]
) -> tuple[float, float]:
    inverse = np.linalg.inv(matrix)
    src = np.array([[[point[0], point[1]]]], dtype=np.float32)
    roi_point = cv2.perspectiveTransform(src, inverse)[0, 0]
    return float(roi_point[0] + offset[0]), float(roi_point[1] + offset[1])


def detect_frame(image: np.ndarray) -> FrameResult:
    tray, offset, tray_bbox = locate_abdomen_window(image)
    body_mask, _ = segment_roach(tray)
    canonical, canonical_mask, matrix = rectify_roach(tray, body_mask)
    candidates, spot_mask = detect_candidates(canonical, canonical_mask)
    tail_end_y = float(np.nonzero(canonical_mask)[0].max())
    pairs = validated_abdominal_sequence(pair_candidates(candidates), tail_end_y)
    target = pairs[-2]
    canonical_points = [(target.left.x, target.left.y), (target.right.x, target.right.y)]
    image_points = [canonical_to_image(point, matrix, offset) for point in canonical_points]
    pair_quality = target.score
    confidence = float(np.clip(0.90 * pair_quality + 0.10, 0.0, 1.0))

    annotated = image.copy()
    for pair in [target]:
        points = [
            canonical_to_image((pair.left.x, pair.left.y), matrix, offset),
            canonical_to_image((pair.right.x, pair.right.y), matrix, offset),
        ]
        color = (0, 255, 0)
        thickness = 6
        p1, p2 = [tuple(int(round(v)) for v in point) for point in points]
        cv2.circle(annotated, p1, 18, color, thickness)
        cv2.circle(annotated, p2, 18, color, thickness)
        cv2.line(annotated, p1, p2, color, thickness)
        label = "P2 target"
        cv2.putText(annotated, label, (min(p1[0], p2[0]), min(p1[1], p2[1]) - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)
    cv2.putText(annotated, f"P2 confidence={confidence:.2f}", (40, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 4)
    return FrameResult(
        points_image=image_points,
        points_canonical=canonical_points,
        pairs=pairs,
        confidence=confidence,
        tray_bbox=tray_bbox,
        canonical=canonical,
        spot_mask=spot_mask,
        annotated=annotated,
    )


def detect_multiframe(images: Iterable[np.ndarray]) -> dict:
    """多帧检测并用中位数抑制相机噪声和瞬时反光。"""
    results: list[FrameResult] = []
    errors: list[str] = []
    for image in images:
        try:
            results.append(detect_frame(image))
        except ValueError as exc:
            errors.append(str(exc))
    if not results:
        raise ValueError("所有帧均检测失败: " + "; ".join(errors))

    points = np.array([result.points_image for result in results], dtype=np.float32)
    median_points = np.median(points, axis=0)
    deviations = np.linalg.norm(points - median_points[None, :, :], axis=2).mean(axis=1)
    stable = deviations <= max(8.0, float(np.median(deviations) + 2.5 * np.std(deviations)))
    if stable.any():
        median_points = np.median(points[stable], axis=0)
    stability = float(np.exp(-float(np.median(deviations)) / 12.0))
    confidence = float(np.mean([r.confidence for r in results]))
    confidence *= stability
    center = median_points.mean(axis=0)
    vector = median_points[1] - median_points[0]
    angle_deg = float(np.degrees(np.arctan2(vector[1], vector[0])) % 360.0)
    return {
        "target_name": "P2",
        "target_definition": "abdominal_penultimate_pair",
        "coordinate_system": "full_image_pixels",
        "points": median_points.tolist(),
        "center": center.tolist(),
        "angle_deg": round(angle_deg, 4),
        "confidence": round(confidence, 4),
        "successful_frames": len(results),
        "failed_frames": len(errors),
        "stable_frames": int(stable.sum()),
        "frame_results": results,
        "errors": errors,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="检测蟑螂腹部倒数第二对黑点 P2")
    parser.add_argument("images", nargs="+", help="一张或多张输入图片")
    parser.add_argument("--output-dir", default="vision_output", help="调试结果目录")
    args = parser.parse_args()

    loaded = [imread_unicode(path) for path in args.images]
    result = detect_multiframe(loaded)
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    for index, frame in enumerate(result["frame_results"]):
        imwrite_unicode(output / f"frame_{index:02d}_annotated.jpg", frame.annotated)
        imwrite_unicode(output / f"frame_{index:02d}_canonical.jpg", frame.canonical)
        imwrite_unicode(output / f"frame_{index:02d}_spot_mask.png", frame.spot_mask)

    serializable = {key: value for key, value in result.items() if key != "frame_results"}
    (output / "result.json").write_text(
        json.dumps(serializable, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(serializable, ensure_ascii=False, indent=2))
    return 0 if result["confidence"] >= 0.55 else 2


if __name__ == "__main__":
    raise SystemExit(main())
