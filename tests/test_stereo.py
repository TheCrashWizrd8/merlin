"""Unit tests for parallel-camera triangulation."""

import math

from src.stereo import StereoConfig, focal_length_px, pair_tracks, triangulate
from src.tracker import TrackResult


def _track(x: int, y: int, detected: bool = True) -> TrackResult:
    return TrackResult(
        apple_detected=detected,
        target_x=x,
        target_y=y,
        bbox_x1=x - 10,
        bbox_y1=y - 10,
        bbox_x2=x + 10,
        bbox_y2=y + 10,
        confidence=0.9,
    )


def test_focal_length_from_67deg():
    f = focal_length_px(640, 67.0, diagonal=False)
    # f = 320 / tan(33.5°) ≈ 483.4
    assert 480.0 < f < 487.0


def test_focal_length_67deg_diagonal():
    f = focal_length_px(640, 67.0, 480, diagonal=True)
    # diagonal 800 px; f = 400 / tan(33.5°) ≈ 605
    assert 600.0 < f < 612.0


def test_range_on_axis_one_metre():
    cfg = StereoConfig(baseline_m=0.16, fov_h_deg=67.0, fov_is_diagonal=False)
    f = focal_length_px(640, 67.0, diagonal=False)
    z = 1.0
    disparity = f * cfg.baseline_m / z
    # Object on the stereo midpoint: left x = cx + d/2, right x = cx - d/2
    cx, cy = 320, 240
    left = _track(int(round(cx + disparity / 2.0)), cy)
    right = _track(int(round(cx - disparity / 2.0)), cy)
    result = triangulate(left, right, 640, 480, cfg)
    assert result.ok
    assert result.range_m is not None
    assert abs(result.range_m - 1.0) < 0.03
    assert result.z_m is not None
    assert abs(result.z_m - 1.0) < 0.03


def test_closer_object_has_larger_disparity():
    cfg = StereoConfig(baseline_m=0.16, fov_h_deg=67.0)
    far = triangulate(_track(360, 240), _track(280, 240), 640, 480, cfg)
    near = triangulate(_track(400, 240), _track(240, 240), 640, 480, cfg)
    assert far.ok and near.ok
    assert near.range_m < far.range_m
    assert near.disparity_px > far.disparity_px


def test_y_mismatch_rejected():
    cfg = StereoConfig(match_max_dy_px=40)
    result = triangulate(_track(360, 100), _track(280, 200), 640, 480, cfg)
    assert not result.ok
    assert "y_mismatch" in result.reason


def test_negative_disparity_suggests_swap():
    cfg = StereoConfig()
    result = triangulate(_track(200, 240), _track(400, 240), 640, 480, cfg)
    assert not result.ok
    assert "negative_disparity" in result.reason


def test_pair_tracks_requires_both():
    assert pair_tracks(_track(320, 240), _track(300, 240), 40)
    assert not pair_tracks(_track(320, 240, detected=False), _track(300, 240), 40)


def test_fuse_tracks_averages_when_paired():
    from src.stereo import fuse_tracks

    left = _track(400, 200)
    left.error_x = 0.4
    right = _track(240, 200)
    right.error_x = -0.2
    fused = fuse_tracks(left, right, paired=True)
    assert fused.apple_detected
    assert abs(fused.error_x - 0.1) < 1e-6


def test_known_geometry_matches_pinhole():
    """Invert the pinhole formula: given Z and X, recover the same Z."""
    cfg = StereoConfig(baseline_m=0.16, fov_h_deg=67.0, fov_is_diagonal=False)
    f = focal_length_px(640, 67.0, diagonal=False)
    z = 1.5
    x_left = 0.10  # 10 cm right of left camera
    x_px = 320 + x_left * f / z
    d = f * cfg.baseline_m / z
    left = _track(int(round(x_px)), 240)
    right = _track(int(round(x_px - d)), 240)
    result = triangulate(left, right, 640, 480, cfg)
    assert result.ok
    assert result.z_m is not None
    assert abs(result.z_m - z) < 0.04


def test_diagonal_fov_recovers_50cm():
    cfg = StereoConfig(baseline_m=0.16, fov_h_deg=67.0, fov_is_diagonal=True)
    f = focal_length_px(640, 67.0, 480, diagonal=True)
    z = 0.50
    d = f * cfg.baseline_m / z
    cx, cy = 320, 240
    left = _track(int(round(cx + d / 2.0)), cy)
    right = _track(int(round(cx - d / 2.0)), cy)
    result = triangulate(left, right, 640, 480, cfg)
    assert result.ok
    assert abs(result.range_m - 0.50) < 0.03


def test_horizontal_67_underreads_true_50cm():
    """67° as horizontal FOV is what made a 50 cm apple read ~38 cm."""
    f_true = focal_length_px(640, 67.0, 480, diagonal=True)
    d = f_true * 0.16 / 0.50
    cx, cy = 320, 240
    left = _track(int(round(cx + d / 2.0)), cy)
    right = _track(int(round(cx - d / 2.0)), cy)
    wrong = StereoConfig(baseline_m=0.16, fov_h_deg=67.0, fov_is_diagonal=False)
    result = triangulate(left, right, 640, 480, wrong)
    assert result.ok
    assert 0.35 < result.range_m < 0.42


if __name__ == "__main__":
    test_focal_length_from_67deg()
    test_focal_length_67deg_diagonal()
    test_range_on_axis_one_metre()
    test_closer_object_has_larger_disparity()
    test_y_mismatch_rejected()
    test_negative_disparity_suggests_swap()
    test_pair_tracks_requires_both()
    test_fuse_tracks_averages_when_paired()
    test_known_geometry_matches_pinhole()
    test_diagonal_fov_recovers_50cm()
    test_horizontal_67_underreads_true_50cm()
    print("ok")
