"""Vision overlay API payload."""

from src.vision_state import clear_vision, update_vision, vision_snapshot


def test_vision_snapshot_cleared():
    clear_vision()
    snap = vision_snapshot()
    assert snap["running"] is False
    assert snap["fov"]["objects"] == []


def test_vision_update_tracks():
    clear_vision()

    class _Track:
        apple_detected = True
        label = "apple"
        confidence = 0.9
        bbox_x1 = 10
        bbox_y1 = 20
        bbox_x2 = 30
        bbox_y2 = 40
        target_x = 20
        target_y = 30

    update_vision(
        model_id="detect",
        backend="hailo",
        running=True,
        left_shape=(480, 640, 3),
        track_left=_Track(),
        stereo_ok=True,
        range_m=1.5,
    )
    snap = vision_snapshot()
    assert snap["running"] is True
    assert snap["left"]["track"]["x1"] == 10
    assert snap["fov"]["track"]["x1"] == 10
    assert snap["stereo"]["range_m"] == 1.5
    assert snap["overlay"]["fov_from_stereo_ready"] is True


def test_fov_box_is_normalized_average():
    clear_vision()

    class _Left:
        apple_detected = True
        label = "gate"
        confidence = 0.8
        bbox_x1 = 0
        bbox_y1 = 0
        bbox_x2 = 100
        bbox_y2 = 100
        target_x = 50
        target_y = 50

    class _Right:
        apple_detected = True
        label = "gate"
        confidence = 0.9
        bbox_x1 = 100
        bbox_y1 = 0
        bbox_x2 = 200
        bbox_y2 = 100
        target_x = 150
        target_y = 50

    update_vision(
        running=True,
        left_shape=(200, 200, 3),
        right_shape=(200, 200, 3),
        fov_shape=(400, 400, 3),
        track_left=_Left(),
        track_right=_Right(),
    )
    box = vision_snapshot()["fov"]["track"]
    assert box["x1"] == 100
    assert box["x2"] == 300
    assert box["y1"] == 0
    assert box["y2"] == 200
    assert box["cx"] == 200
    assert box["cy"] == 100
