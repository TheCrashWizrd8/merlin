"""Tests for detection dropout hold in controller.compute()."""

from src.controller import Controller
from src.telemetry_context import TelemetryContext
from src.tracker import TrackResult


def _track(error_x: float = 0.4, confidence: float = 0.8) -> TrackResult:
    return TrackResult(
        apple_detected=True,
        target_x=400,
        target_y=300,
        bbox_x1=380,
        bbox_y1=280,
        bbox_x2=420,
        bbox_y2=320,
        bbox_width=40,
        bbox_height=40,
        bbox_area=1600,
        frame_area=640 * 480,
        chosen_label="apple",
        error_x=error_x,
        error_y=0.0,
        confidence=confidence,
    )


def test_controller_holds_commands_on_missed_frame():
    ctrl = Controller(
        deadzone=0.02,
        min_steer_command=0.25,
        min_drive_command=0.55,
        hold_missed_frames=4,
        use_stereo_range=False,
    )
    tel = TelemetryContext.empty()
    good = _track()
    out1 = ctrl.compute(good, telemetry=tel)
    assert out1.drive_motor != 0.0 or out1.steering_servo != 0.0

    missed = TrackResult(apple_detected=False)
    out2 = ctrl.compute(missed, telemetry=tel)
    assert out2.steering_servo == out1.steering_servo
    assert out2.drive_motor == out1.drive_motor
    assert out2.camera_tilt_servo == out1.camera_tilt_servo
    assert out2.apple_detected is True


def test_controller_stops_after_hold_expires():
    ctrl = Controller(
        deadzone=0.02,
        min_steer_command=0.25,
        min_drive_command=0.55,
        hold_missed_frames=2,
        use_stereo_range=False,
    )
    tel = TelemetryContext.empty()
    ctrl.compute(_track(), telemetry=tel)
    missed = TrackResult(apple_detected=False)
    ctrl.compute(missed, telemetry=tel)
    ctrl.compute(missed, telemetry=tel)
    out = ctrl.compute(missed, telemetry=tel)
    assert out.steering_servo == 0.0
    assert out.drive_motor == 0.0
    assert out.apple_detected is False
