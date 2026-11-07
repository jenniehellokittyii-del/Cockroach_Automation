"""Locate two visible body parts inside a light fixture without fixed ROIs."""
import cv2
import numpy as np


def filled_largest(mask):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("No foreground contour found")
    result = np.zeros_like(mask)
    cv2.drawContours(result, [max(contours, key=cv2.contourArea)], -1, 255, -1)
    return result


def locate_parts(image):
    h, w = image.shape[:2]
    # Work at a fixed scale so morphology does not depend on photo resolution.
    scale = min(1., 900 / max(h, w))
    small = cv2.resize(image, None, fx=scale, fy=scale)
    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    _, light = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    board = filled_largest(light)
    if np.count_nonzero(board) < .15 * board.size:
        raise ValueError("Cannot identify the light fixture")
    # Exclude fixture edges and dark surroundings, including screw holes near edges.
    board = cv2.erode(board, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (45, 45)))
    candidate = np.uint8(((gray < 75) | ((hsv[:, :, 1] > 110) & (gray < 180))) & (board > 0)) * 255
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    candidate = cv2.morphologyEx(candidate, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(candidate)
    valid = [i for i in range(1, count) if stats[i, 4] > .008 * board.size
             and min(stats[i, 2:4]) > .05 * min(gray.shape)]
    valid.sort(key=lambda i: stats[i, 4], reverse=True)
    if len(valid) < 2:
        raise ValueError("Cannot separate two visible body parts; check lighting/occlusion")
    valid = valid[:2]
    # Brown segmented abdomen has substantially more orange/brown than dark head.
    scores = [float(np.mean((hsv[:, :, 0][labels == i] < 30) &
                            (hsv[:, :, 1][labels == i] > 100) &
                            (hsv[:, :, 2][labels == i] > 85))) for i in valid]
    if abs(scores[0] - scores[1]) < .08:
        raise ValueError("Head/abdomen appearance is ambiguous; manual review required")
    abdomen_index = int(np.argmax(scores))
    result = {}
    for j, i in enumerate(valid):
        name = "abdomen" if j == abdomen_index else "head"
        component = np.uint8(labels == i) * 255
        x, y, bw, bh = stats[i, :4]
        pad = int(.08 * max(bw, bh)) + 3
        box = (max(0, int((x-pad)/scale)), max(0, int((y-pad)/scale)),
               min(w, int((x+bw+pad)/scale)), min(h, int((y+bh+pad)/scale)))
        full_seed = cv2.resize(component, (w, h), interpolation=cv2.INTER_NEAREST)
        x1, y1, x2, y2 = box
        crop = image[y1:y2, x1:x2]
        rough = full_seed[y1:y2, x1:x2]
        radius = max(3, int(.015 * min(crop.shape[:2])))
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2*radius+1, 2*radius+1))
        seeds = np.full(rough.shape, cv2.GC_BGD, np.uint8)
        seeds[cv2.dilate(rough, kernel) > 0] = cv2.GC_PR_BGD
        seeds[rough > 0] = cv2.GC_PR_FGD
        seeds[cv2.erode(rough, kernel) > 0] = cv2.GC_FGD
        if not np.any(seeds == cv2.GC_FGD):
            raise ValueError(f"{name}: foreground seed too small")
        cv2.grabCut(crop, seeds, None, np.zeros((1, 65), np.float64),
                    np.zeros((1, 65), np.float64), 5, cv2.GC_INIT_WITH_MASK)
        mask = filled_largest(np.uint8((seeds == 1) | (seeds == 3)) * 255)
        result[name] = {"box": box, "mask": mask, "brown_fraction": scores[j]}
    a = np.array(result["head"]["box"]).reshape(2, 2).mean(axis=0)
    b = np.array(result["abdomen"]["box"]).reshape(2, 2).mean(axis=0)
    result["direction_deg"] = float(np.degrees(np.arctan2(*(b-a)[::-1])))
    return result
