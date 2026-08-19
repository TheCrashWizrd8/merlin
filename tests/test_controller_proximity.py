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


def test_falls_back_to_size_when_stereo_misses():
    ctrl = Controller(
        use_stereo_range=True,
        use_size_for_drive=True,
        size_min_ratio=0.001,
        size_max_ratio=0.01,
        range_far_m=2.0,
        range_near_m=0.5,
    )
    out = ctrl.compute(_track(), range_m=None, stereo_ok=False)
    assert out.proximity_t > 0.0
    assert "size" in out.approach_note


if __name__ == "__main__":
    test_proximity_from_stereo_range()
    test_falls_back_to_size_when_stereo_misses()
    print("ok")
