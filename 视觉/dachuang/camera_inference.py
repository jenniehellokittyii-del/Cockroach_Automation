from datetime import datetime
import time
import cv2
from .auto_body import locate_parts
from .six_point_locator import geometry_estimate, locate

def infer(frame, model, mode):
    start = time.monotonic()
    points, error = None, None
    parts, segmentation_error = None, None
    try:
        parts = locate_parts(frame)
    except (ValueError, cv2.error) as exc:
        segmentation_error = str(exc)
    method = "NO DETECTION"
    try:
        if mode == "geometry":
            if parts is None:
                raise ValueError(segmentation_error)
            points = geometry_estimate(frame, model, parts)
            method = "GEOMETRY ESTIMATE - REVIEW"
        else:
            try:
                points, _ = locate(frame, model)
                method = "TEXTURE REGISTRATION"
            except ValueError:
                if mode == "texture":
                    raise
                if parts is None:
                    raise ValueError(segmentation_error)
                points = geometry_estimate(frame, model, parts)
                method = "GEOMETRY ESTIMATE - REVIEW"
    except (ValueError, cv2.error) as exc:
        error = str(exc)
    return {"frame": frame, "points": points, "method": method, "error": error,
            "parts": parts, "segmentation_error": segmentation_error,
            "inference_ms": (time.monotonic()-start)*1000,
            "timestamp": datetime.now().astimezone().isoformat()}

