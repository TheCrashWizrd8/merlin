"""
stereo.py
---------
Parallel-camera triangulation from a pair of YOLO bounding-box centres.

Geometry (pinhole, optical axes parallel, cameras level):

    Z = (f * B) / d
    X = (x_left - cx) * Z / f
    Y = (y_left - cy) * Z / f
    range = sqrt(X^2 + Y^2 + Z^2)

where
    B  = baseline (metres) between camera centres
    d  = disparity = x_left - x_right  (pixels)
    f  = focal length in pixels, from horizontal FOV:
         f = (width / 2) / tan(fov_h / 2)

The reported range is the 3D distance from the **midpoint between the
two cameras** to the apple centre.  X is positive to the right of that
midpoint; Y is positive downward (image convention).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

from src.tracker import TrackResult


PROJECT_ROOT = Path(__file__).parent.parent
HARDWARE_PATH = PROJECT_ROOT / "config" / "hardware.yaml"


@dataclass(frozen=True)
class StereoConfig:
    """Physical + matching settings for a two-camera rig."""

    enabled: bool = False
    left_device: int | str = 0
    right_device: int | str = 2
    baseline_m: float = 0.16
    fov_h_deg: float = 67.0
    fov_is_diagonal: bool = True
    focal_length_px: Optional[float] = None
    match_max_dy_px: int = 40
    min_disparity_px: float = 2.0
    range_scale: float = 1.0


@dataclass(frozen=True)
class StereoResult:
    """Triangulation output for one matched pair (or a miss)."""

    ok: bool
    range_m: Optional[float] = None
    x_m: Optional[float] = None
    y_m: Optional[float] = None
    z_m: Optional[float] = None
    disparity_px: Optional[float] = None
    reason: str = ""


def load_stereo_config(path: Path = HARDWARE_PATH) -> StereoConfig:
    """Read the `cameras:` block from hardware.yaml."""
    try:
        with open(path, "r") as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    cams = cfg.get("cameras") or {}
    num = int(cams.get("num_cameras", 1) or 1)
    baseline_cm = float(cams.get("baseline_cm", 16.0))
    focal = cams.get("focal_length_px")
    return StereoConfig(
        enabled=num >= 2,
        left_device=cams.get("left_device", 0),
        right_device=cams.get("right_device", 2),
        baseline_m=baseline_cm / 100.0,
        fov_h_deg=float(cams.get("fov_h_deg", 67.0)),
        fov_is_diagonal=bool(cams.get("fov_is_diagonal", True)),
        focal_length_px=float(focal) if focal not in (None, "") else None,
        match_max_dy_px=int(cams.get("match_max_dy_px", 40)),
        min_disparity_px=float(cams.get("min_disparity_px", 2.0)),
        range_scale=float(cams.get("range_scale", 1.0) or 1.0),
    )


def focal_length_px(
    frame_width: int,
    fov_h_deg: float,
    frame_height: int = 480,
    diagonal: bool = False,
) -> float:
    """Convert FOV (degrees) to focal length in pixels.

    USB camera specs are usually **diagonal**.  Treating that number as
    horizontal under-reads range (e.g. 50 cm shown as ~38 cm).
    """
    if frame_width <= 0:
        raise ValueError("frame_width must be positive")
    if fov_h_deg <= 0.0 or fov_h_deg >= 180.0:
        raise ValueError("fov_h_deg must be in (0, 180)")
    if diagonal:
        if frame_height <= 0:
            raise ValueError("frame_height must be positive")
        half_sensor = math.hypot(frame_width, frame_height) / 2.0
    else:
        half_sensor = frame_width / 2.0
    half = math.radians(fov_h_deg) / 2.0
    return half_sensor / math.tan(half)


def pair_tracks(
    left: TrackResult,
    right: TrackResult,
    max_dy_px: int,
) -> bool:
    """
    True if both cameras have an apple and the bbox centres line up
    well enough in Y to be the same object.
    """
    if not left.apple_detected or not right.apple_detected:
        return False
    return abs(left.target_y - right.target_y) <= max_dy_px


def triangulate(
    left: TrackResult,
    right: TrackResult,
    frame_width: int,
    frame_height: int,
    cfg: StereoConfig,
) -> StereoResult:
    """
    Estimate 3D position from a left/right TrackResult pair.

    Uses bbox centres.  Returns StereoResult.ok=False when the pair
    cannot be triangulated (missing detection, Y mismatch, or tiny /
    inverted disparity).
    """
    if not left.apple_detected and not right.apple_detected:
        return StereoResult(ok=False, reason="no_detection")
    if not left.apple_detected:
        return StereoResult(ok=False, reason="left_miss")
    if not right.apple_detected:
        return StereoResult(ok=False, reason="right_miss")
    if not pair_tracks(left, right, cfg.match_max_dy_px):
        return StereoResult(
            ok=False,
            reason=f"y_mismatch dy={abs(left.target_y - right.target_y)}px",
        )

    disparity = float(left.target_x - right.target_x)
    if disparity < cfg.min_disparity_px:
        if disparity < 0:
            return StereoResult(
                ok=False,
                disparity_px=disparity,
                reason="negative_disparity (swap left/right devices?)",
            )
        return StereoResult(
            ok=False,
            disparity_px=disparity,
            reason="disparity_too_small",
        )

    f = (
        cfg.focal_length_px
        if cfg.focal_length_px is not None
        else focal_length_px(
            frame_width,
            cfg.fov_h_deg,
            frame_height=frame_height,
            diagonal=cfg.fov_is_diagonal,
        )
    )
    if f <= 0.0:
        return StereoResult(ok=False, reason="bad_focal_length")

    z_m = (f * cfg.baseline_m) / disparity
    # Left-camera frame, then shift X so origin is the stereo midpoint.
    x_left_m = (left.target_x - frame_width / 2.0) * z_m / f
    y_m = (left.target_y - frame_height / 2.0) * z_m / f
    x_mid_m = x_left_m - cfg.baseline_m / 2.0
    range_m = math.sqrt(x_mid_m * x_mid_m + y_m * y_m + z_m * z_m)
    scale = cfg.range_scale if cfg.range_scale > 0.0 else 1.0
    range_m *= scale
    z_m *= scale
    x_mid_m *= scale
    y_m *= scale

    if not math.isfinite(range_m) or range_m <= 0.0:
        return StereoResult(ok=False, disparity_px=disparity, reason="non_finite")

    return StereoResult(
        ok=True,
        range_m=range_m,
        x_m=x_mid_m,
        y_m=y_m,
        z_m=z_m,
        disparity_px=disparity,
        reason="ok",
    )


def fuse_tracks(left: TrackResult, right: TrackResult, paired: bool) -> TrackResult:
    """
    Build the track used for steering / tilt.

    When both cameras see a matched apple, average the normalised errors
    so control sits on the stereo midpoint.  Otherwise use whichever
    camera has a detection (left preferred).
    """
    if paired and left.apple_detected and right.apple_detected:
        return TrackResult(
            apple_detected=True,
            target_x=(left.target_x + right.target_x) // 2,
            target_y=(left.target_y + right.target_y) // 2,
            bbox_x1=left.bbox_x1,
            bbox_y1=left.bbox_y1,
            bbox_x2=left.bbox_x2,
            bbox_y2=left.bbox_y2,
            bbox_width=left.bbox_width,
            bbox_height=left.bbox_height,
            bbox_area=left.bbox_area,
            frame_area=left.frame_area,
            chosen_label=left.chosen_label or right.chosen_label,
            error_x=(left.error_x + right.error_x) / 2.0,
            error_y=(left.error_y + right.error_y) / 2.0,
            confidence=min(left.confidence, right.confidence),
        )
    if left.apple_detected:
        return left
    if right.apple_detected:
        return right
    return TrackResult(apple_detected=False)


def compose_side_by_side(
    left_bgr,
    right_bgr,
    left_label: str = "L",
    right_label: str = "R",
):
    """Stack two BGR frames horizontally, letterboxing if heights differ."""
    import cv2
    import numpy as np

    lh, lw = left_bgr.shape[:2]
    rh, rw = right_bgr.shape[:2]
    height = max(lh, rh)
    if lh != height:
        pad = height - lh
        left_bgr = cv2.copyMakeBorder(left_bgr, 0, pad, 0, 0, cv2.BORDER_CONSTANT)
    if rh != height:
        pad = height - rh
        right_bgr = cv2.copyMakeBorder(right_bgr, 0, pad, 0, 0, cv2.BORDER_CONSTANT)
    canvas = np.hstack((left_bgr, right_bgr))
    cv2.putText(
        canvas, left_label, (8, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2, cv2.LINE_AA,
    )
    cv2.putText(
        canvas, right_label, (lw + 8, 28),
        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 220, 255), 2, cv2.LINE_AA,
    )
    return canvas
