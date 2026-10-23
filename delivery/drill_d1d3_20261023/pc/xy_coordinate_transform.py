# -*- coding: utf-8 -*-
"""将完整4K图像中的像素误差换算为XY轴步数/毫米修正量。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


DEFAULT_CALIBRATION = Path(__file__).with_name("xy_pixel_calibration.json")
XY_STEPS_PER_MM = 200.0 / 12.0


def load_calibration(path: str | Path = DEFAULT_CALIBRATION) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def pixel_error_to_xy(
    current_pixel: tuple[float, float],
    desired_pixel: tuple[float, float] | None = None,
    calibration: dict | None = None,
) -> dict:
    """计算把当前像素点移到期望像素点所需的XY相对运动。"""
    calibration = calibration or load_calibration()
    desired = np.asarray(
        desired_pixel or calibration["reference_p2_center_px"], dtype=np.float64
    )
    current = np.asarray(current_pixel, dtype=np.float64)
    pixel_error = desired - current
    matrix = np.asarray(calibration["steps_per_pixel"], dtype=np.float64)
    step_delta = matrix @ pixel_error
    return {
        "pixel_error": pixel_error.tolist(),
        "step_delta_float": step_delta.tolist(),
        "step_delta": np.rint(step_delta).astype(int).tolist(),
        "mm_delta": (step_delta / XY_STEPS_PER_MM).tolist(),
    }


def pixel_to_absolute_xy(
    current_pixel: tuple[float, float],
    current_steps: tuple[int, int],
    desired_pixel: tuple[float, float] | None = None,
    calibration: dict | None = None,
) -> dict:
    """由当前P2像素和当前机械位置，计算对准参考点的绝对XY坐标。"""
    relative = pixel_error_to_xy(current_pixel, desired_pixel, calibration)
    absolute_float = np.asarray(current_steps, dtype=np.float64) + np.asarray(
        relative["step_delta_float"], dtype=np.float64
    )
    absolute_steps = np.rint(absolute_float).astype(int)
    return {
        **relative,
        "current_steps": list(current_steps),
        "absolute_steps_float": absolute_float.tolist(),
        "absolute_steps": absolute_steps.tolist(),
        "absolute_mm": (absolute_steps / XY_STEPS_PER_MM).tolist(),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="P2像素坐标转XY修正量")
    parser.add_argument("u", type=float, help="当前P2中点u像素")
    parser.add_argument("v", type=float, help="当前P2中点v像素")
    parser.add_argument("--calibration", default=str(DEFAULT_CALIBRATION))
    parser.add_argument("--machine-x", type=int, help="当前机械X绝对步数")
    parser.add_argument("--machine-y", type=int, help="当前机械Y绝对步数")
    args = parser.parse_args()
    calibration = load_calibration(args.calibration)
    if (args.machine_x is None) != (args.machine_y is None):
        parser.error("--machine-x 和 --machine-y 必须同时提供")
    if args.machine_x is None:
        result = pixel_error_to_xy((args.u, args.v), calibration=calibration)
    else:
        result = pixel_to_absolute_xy(
            (args.u, args.v), (args.machine_x, args.machine_y), calibration=calibration
        )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
