"""Single-example landmark transfer using SIFT and separate part homographies.

Build:   python six_point_locator.py build
Predict: python six_point_locator.py predict new_photo.jpg
Check:   python six_point_locator.py check
Coordinates are original image pixels, origin at top left; pairs are left/right.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from .auto_body import locate_parts

BASE = Path(__file__).resolve().parent
MODEL = BASE / "six_point_model.npz"


def body_frames(image, parts=None):
    if parts is None:
        parts = locate_parts(image)
    centers = {}
    clouds = {}
    for name in ("head", "abdomen"):
        y, x = np.nonzero(parts[name]["mask"])
        cloud = np.column_stack([x, y]).astype(float) + np.array(parts[name]["box"][:2])
        clouds[name] = cloud
        centers[name] = cloud.mean(axis=0)
    axis = centers["abdomen"] - centers["head"]
    axis /= np.linalg.norm(axis)
    sideways = np.array([axis[1], -axis[0]])
    basis = np.column_stack([sideways, axis])
    return {name: (cloud @ basis, basis) for name, cloud in clouds.items()}


def cross_section(cloud, longitudinal):
    band = max(2., .012 * np.ptp(cloud[:, 1]))
    xs = cloud[np.abs(cloud[:, 1] - longitudinal) <= band, 0]
    if len(xs) < 5:
        raise ValueError("Insufficient contour support near a landmark")
    return float(xs.min()), float(xs.max())


def geometry_template(image, points):
    frames = body_frames(image)
    normalized = np.zeros((6, 2), float)
    for name, ids in [("head", range(4)), ("abdomen", range(4, 6))]:
        cloud, basis = frames[name]
        low, high = cloud[:, 1].min(), cloud[:, 1].max()
        for i in ids:
            x, y = points[i] @ basis
            left, right = cross_section(cloud, y)
            normalized[i] = ((x-left)/(right-left), (y-low)/(high-low))
    if np.any(normalized < 0) or np.any(normalized > 1):
        raise ValueError("Reference marks must be inside the segmented body")
    return normalized


def geometry_estimate(image, model, parts=None):
    if "geometry_template" not in model:
        raise ValueError("Model lacks geometry data; run the build command first")
    frames = body_frames(image, parts)
    points = np.zeros((6, 2), np.float32)
    for name, ids in [("head", range(4)), ("abdomen", range(4, 6))]:
        cloud, basis = frames[name]
        low, high = cloud[:, 1].min(), cloud[:, 1].max()
        if bool(np.asarray(model.get('symmetric_pairs', False)).item()):
            # 两个身体区域质心的连线为共同中心轴；其横向投影相同。
            center_x = float(cloud[:, 0].mean())
            indices = list(ids)
            for first, second in zip(indices[::2], indices[1::2]):
                pair = model['geometry_template'][[first, second]]
                y = low + float(pair[:, 1].mean()) * (high-low)
                left, right = cross_section(cloud, y)
                radius = abs(float(pair[0, 0] - pair[1, 0])) * (right-left) / 2
                radius = min(radius, center_x-left, right-center_x)
                if radius <= 0:
                    raise ValueError('Body center axis lies outside landmark cross section')
                sign = 1 if pair[0, 0] >= pair[1, 0] else -1
                # 仅成对缩小间距，避免独立吸附破坏轴对称关系。
                for factor in np.linspace(1., .2, 41):
                    guesses = np.array([[center_x + sign*radius*factor, y],
                                        [center_x - sign*radius*factor, y]])
                    distances = [np.min(np.linalg.norm(cloud-guess, axis=1))
                                 for guess in guesses]
                    if max(distances) <= 1.5:
                        points[[first, second]] = guesses @ basis.T
                        break
                else:
                    raise ValueError('Cannot place symmetric pair inside body foreground')
            continue
        for i in ids:
            fraction, along = model["geometry_template"][i]
            y = low + along * (high-low)
            left, right = cross_section(cloud, y)
            guess = np.array([left + fraction*(right-left), y])
            # Snap to foreground so concave contour gaps do not receive a point.
            nearest = cloud[np.argmin(np.sum((cloud-guess)**2, axis=1))]
            points[i] = nearest @ basis.T
    return points


def read_image(path):
    image = cv2.imdecode(np.fromfile(path, np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Cannot read image: {path}")
    return image


def write_image(path, image):
    ok, data = cv2.imencode(Path(path).suffix, image)
    if not ok:
        raise RuntimeError(f"Cannot encode {path}")
    data.tofile(path)


def features(image, mask=None):
    keypoints, descriptors = cv2.SIFT_create(nfeatures=10000, contrastThreshold=0.015).detectAndCompute(
        cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), mask)
    if descriptors is None:
        raise ValueError("No usable image features")
    return np.float32([p.pt for p in keypoints]), descriptors


def build(reference, annotation, model):
    clean, marked = read_image(reference), read_image(annotation)
    if clean.shape != marked.shape:
        raise ValueError("Reference and annotation must have the same dimensions")
    b, g, r = cv2.split(marked)
    # Bright pure red pen; natural reddish brown is deliberately excluded.
    red = np.uint8((r > 220) & (g < 65) & (b < 65)) * 255
    count, _, stats, centers = cv2.connectedComponentsWithStats(red)
    points = [centers[i] for i in range(1, count) if 8 <= stats[i, cv2.CC_STAT_AREA] <= 250]
    if len(points) != 6:
        raise ValueError(f"Expected six red marks, found {len(points)}")
    points = np.array(sorted(points, key=lambda p: p[1]), np.float32)
    points = np.concatenate([pair[np.argsort(pair[:, 0])] for pair in points.reshape(3, 2, 2)])
    height, width = clean.shape[:2]
    data = {"points": points, "size": np.array([width, height]),
            "geometry_template": geometry_template(clean, points)}
    # Each visible part can move independently. Restrict reference features to
    # the animal so the white fixture cannot dominate registration.
    for part, ids in [("upper", [0, 1, 2, 3]), ("abdomen", [4, 5])]:
        selected = points[ids]
        lo, hi = selected.min(axis=0), selected.max(axis=0)
        spread = max(hi[0] - lo[0], 80)
        x1, x2 = max(0, int(lo[0] - .25 * spread)), min(width, int(hi[0] + .25 * spread))
        y1 = max(0, int(lo[1] - (.6 if part == "upper" else .85) * spread))
        y2 = min(height, int(hi[1] + .3 * spread))
        mask = np.zeros((height, width), np.uint8)
        mask[y1:y2, x1:x2] = 255
        xy, desc = features(clean, mask)
        data[part + "_xy"], data[part + "_desc"] = xy, desc
    np.savez_compressed(model, **data)
    print(f"Saved {model}; numbered annotation points: {points.round(1).tolist()}")


def locate(image, model):
    target_xy, target_desc = features(image)
    output, diagnostics = np.zeros((6, 2), np.float32), {}
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    for part, ids in [("upper", [0, 1, 2, 3]), ("abdomen", [4, 5])]:
        source_xy, desc = model[part + "_xy"], model[part + "_desc"]
        matches = matcher.knnMatch(desc, target_desc, k=2)
        good = [m for pair in matches if len(pair) == 2 for m, n in [pair] if m.distance < .7 * n.distance]
        # A target feature may support only one correspondence.
        unique = {}
        for m in sorted(good, key=lambda m: m.distance):
            unique.setdefault(m.trainIdx, m)
        good = list(unique.values())
        if len(good) < 10:
            raise ValueError(f"{part}: only {len(good)} matches; cannot reliably locate landmarks")
        src = np.float32([source_xy[m.queryIdx] for m in good])
        dst = np.float32([target_xy[m.trainIdx] for m in good])
        matrix, inliers = cv2.findHomography(src, dst, cv2.RANSAC, 4.0)
        if matrix is None or inliers is None:
            raise ValueError(f"{part}: geometric fit failed")
        keep = inliers.ravel().astype(bool)
        if keep.sum() < 10 or keep.mean() < .45:
            raise ValueError(f"{part}: insufficient geometrically consistent matches")
        # Reject fits supported by a narrow strip or a tiny texture patch.
        support = cv2.contourArea(cv2.convexHull(src[keep]))
        extent = np.ptp(source_xy, axis=0)
        if support < .05 * float(np.prod(extent)):
            raise ValueError(f"{part}: matching features cover too little of the body")
        projected = cv2.perspectiveTransform(src[keep, None], matrix)[:, 0]
        residual = np.linalg.norm(projected - dst[keep], axis=1)
        points = cv2.perspectiveTransform(model["points"][ids, None], matrix)[:, 0]
        if not np.isfinite(points).all() or np.any(points < 0) or np.any(points[:, 0] >= image.shape[1]) or np.any(points[:, 1] >= image.shape[0]):
            raise ValueError(f"{part}: inferred points lie outside the image")
        output[ids] = points
        diagnostics[part] = {"matches": len(good), "inliers": int(keep.sum()),
                             "median_reprojection_error_px": float(np.median(residual))}
    return output, diagnostics


def predict(path, model_path, output_dir, strict=False):
    image = read_image(path)
    method = "single_example_part_registration"
    with np.load(model_path, allow_pickle=False) as model:
        try:
            points, diagnostics = locate(image, model)
        except ValueError as exc:
            if strict:
                raise
            points = geometry_estimate(image, model)
            method = "contour_relative_geometry_estimate"
            diagnostics = {"registration_failure": str(exc),
                           "warning": "Estimated relative positions, not detected anatomical features; manual review required."}
    output_dir.mkdir(parents=True, exist_ok=True)
    overlay = image.copy()
    if method == "contour_relative_geometry_estimate":
        cv2.putText(overlay, "GEOMETRY ESTIMATE - REVIEW", (15, 35),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (0, 165, 255), 2)
    for i, (x, y) in enumerate(points, 1):
        p = (int(round(x)), int(round(y)))
        cv2.circle(overlay, p, 8, (0, 0, 255), 2)
        cv2.putText(overlay, str(i), (p[0] + 11, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
    result = {"image": str(path.resolve()), "method": method,
              "points": [{"id": i, "x": float(x), "y": float(y)} for i, (x, y) in enumerate(points, 1)],
              "diagnostics": diagnostics,
              "note": "Fit errors measure feature registration, not anatomical landmark accuracy."}
    name = path.stem + "_six_points"
    write_image(output_dir / (name + ".png"), overlay)
    (output_dir / (name + ".json")).write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))


def check(reference, model_path):
    image = read_image(reference)
    h, w = image.shape[:2]
    # Check known coordinate changes, rather than only matching the source to itself.
    rotation = cv2.getRotationMatrix2D((w / 2, h / 2), 12, .85)
    rotation[:, 2] += (25, -15)
    transform = np.vstack([rotation, [0, 0, 1]])
    changed = cv2.warpPerspective(image, transform, (w, h))
    with np.load(model_path, allow_pickle=False) as model:
        expected = cv2.perspectiveTransform(model["points"][:, None], transform)[:, 0]
        actual, _ = locate(changed, model)
    errors = np.linalg.norm(actual - expected, axis=1)
    print("Synthetic rotation/scale/translation errors (pixels):", errors.round(3).tolist())
    if errors.max() > 5:
        raise RuntimeError("Synthetic geometry check failed")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["build", "predict", "check"])
    parser.add_argument("image", nargs="?", type=Path, default=BASE / "cockroach.jpg")
    parser.add_argument("--annotation", type=Path, default=BASE / "cockroach(1).jpg")
    parser.add_argument("--model", type=Path, default=MODEL)
    parser.add_argument("--output", type=Path, default=BASE / "landmark_results")
    parser.add_argument("--strict", action="store_true", help="Require texture registration; disable geometric estimates")
    args = parser.parse_args()
    if args.command == "build":
        build(args.image, args.annotation, args.model)
    elif args.command == "check":
        check(args.image, args.model)
    else:
        predict(args.image, args.model, args.output, args.strict)


if __name__ == "__main__":
    main()
