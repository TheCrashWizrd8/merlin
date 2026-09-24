"""Controller proximity from stereo range."""

from src.controller import Controller
from src.tracker import TrackResult


def _track() -> TrackResult:
    return TrackResult(
        apple_detected=True,
        target_x=320,
        target_y=240,
        bbox_x1=300,
        bbox_y1=220,
        bbox_x2=340,
        bbox_y2=260,
        bbox_width=40,
        bbox_height=40,
        bbox_area=1600,
        frame_area=640 * 480,
        confidence=0.9,
    )


def test_proximity_from_stereo_range():
    ctrl = Controller(use_stereo_range=True, range_far_m=2.0, range_near_m=0.5)
    out = ctrl.compute(_track(), range_m=1.25, stereo_ok=True)
    assert out.stereo_ok
    assert abs(out.proximity_t - 0.5) < 1e-6
    assert "range" in out.approach_note
    assert "size" not in out.approach_note


def test_stereo_miss_does_not_use_size():
    ctrl = Controller(
        use_stereo_range=True,
        use_size_for_drive=True,
        range_hold_frames=0,
        range_far_m=2.0,
        range_near_m=0.5,
    )
    out = ctrl.compute(_track(), range_m=None, stereo_ok=False)
    assert out.proximity_t == 0.0
    assert "size" not in out.approach_note
    assert not out.stereo_ok


def test_size_only_when_stereo_disabled():
    ctrl = Controller(
        use_stereo_range=False,
        use_size_for_drive=True,
        size_min_ratio=0.001,
        size_max_ratio=0.01,
        range_far_m=2.0,
        range_near_m=0.5,
    )
    out = ctrl.compute(_track(), range_m=None, stereo_ok=False)
    assert out.proximity_t > 0.0
    assert "size" in out.approach_note


def test_far_range_still_uses_stereo():
    ctrl = Controller(
        use_stereo_range=True,
        use_size_for_drive=True,
        range_far_m=1.8,
        range_near_m=0.35,
    )
    out = ctrl.compute(_track(), range_m=8.0, stereo_ok=True)
    assert out.stereo_ok
    assert abs(out.range_m - 8.0) < 1e-6
    assert "range" in out.approach_note
    assert "size" not in out.approach_note
    assert out.proximity_t == 0.0


def test_holds_last_stereo_range():
    ctrl = Controller(
        use_stereo_range=True,
        range_hold_frames=12,
        range_smoothing_alpha=1.0,
        range_far_m=2.0,
        range_near_m=0.5,
    )
    first = ctrl.compute(_track(), range_m=1.25, stereo_ok=True)
    assert abs(first.range_m - 1.25) < 1e-6
    held = ctrl.compute(_track(), range_m=None, stereo_ok=False)
    assert held.stereo_ok
    assert abs(held.range_m - 1.25) < 1e-6
    assert "range" in held.approach_note
    assert held.stereo_note == "hold"
    assert abs(held.proximity_t - 0.5) < 1e-6


def test_hold_expires_after_configured_frames():
    ctrl = Controller(
        use_stereo_range=True,
        range_hold_frames=2,
        range_smoothing_alpha=1.0,
        range_far_m=2.0,
        range_near_m=0.5,
    )
    ctrl.compute(_track(), range_m=1.0, stereo_ok=True)
    assert ctrl.compute(_track(), stereo_ok=False).stereo_ok
    assert ctrl.compute(_track(), stereo_ok=False).stereo_ok
    expired = ctrl.compute(_track(), stereo_ok=False)
    assert not expired.stereo_ok
    assert expired.range_m is None


if __name__ == "__main__":
    test_proximity_from_stereo_range()
    test_stereo_miss_does_not_use_size()
    test_size_only_when_stereo_disabled()
    test_far_range_still_uses_stereo()
    test_holds_last_stereo_range()
    test_hold_expires_after_configured_frames()
    print("ok")
