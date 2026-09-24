"""Unit tests for layered sub motion + ballast height trim."""

from dataclasses import dataclass

from src.sub_motion import (
    SubMotionConfig,
    SubMotionPlanner,
    plan_sub_motion,
    reset_sub_motion_hold,
)
from src.telemetry_context import TelemetryContext


@dataclass
class FakeOutput:
    steering_servo: float = 0.0
    drive_motor: float = 0.0
    camera_tilt_servo: float = 0.0
    error_x: float = 0.0
    error_y: float = 0.0
    apple_detected: bool = False


def setup_function():
    reset_sub_motion_hold()


def test_fins_counter_roll():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0, linked_flap=False)
    tel = TelemetryContext(roll=10.0, pitch=0.0)
    out = FakeOutput(apple_detected=True)
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.fin_left < 0
    assert r.actuators.fin_right > 0
    assert r.phase == "level"


def test_thruster_gated_when_rolled():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0)
    tel = TelemetryContext(roll=20.0, pitch=0.0)
    out = FakeOutput(drive_motor=1.0, apple_detected=True, error_x=0.0, error_y=0.0)
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.thruster_x < 0.5


def test_ballast_apple_below_fills():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0)
    out = FakeOutput(apple_detected=True, error_y=0.5)
    tel = TelemetryContext.empty()
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.ballast_fore > 0
    assert r.ballast_aft > 0


def test_ballast_apple_above_drains():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0)
    out = FakeOutput(apple_detected=True, error_y=-0.5)
    tel = TelemetryContext.empty()
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.ballast_fore < 0
    assert r.ballast_aft < 0


def test_ballast_deadzone():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0)
    out = FakeOutput(apple_detected=True, error_y=0.05)
    tel = TelemetryContext.empty()
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.ballast_fore == 0.0
    assert r.ballast_aft == 0.0


def test_holds_motion_through_detection_dropout():
    cfg = SubMotionConfig(hold_missed_frames=3, output_smoothing_alpha=0.0)
    planner = SubMotionPlanner()
    tel = TelemetryContext.empty()
    moving = FakeOutput(
        apple_detected=True,
        steering_servo=0.5,
        drive_motor=0.8,
        error_x=0.3,
        error_y=0.0,
    )
    first = plan_sub_motion(moving, telemetry=tel, cfg=cfg, planner=planner)
    assert first.actuators.thruster_x > 0.05

    lost = FakeOutput(apple_detected=False)
    second = plan_sub_motion(lost, telemetry=tel, cfg=cfg, planner=planner)
    assert second.actuators.thruster_x == first.actuators.thruster_x
    assert second.actuators.aft_steer_y == first.actuators.aft_steer_y
    assert "hold" in second.note


def test_centred_target_does_not_latch_thrust():
    cfg = SubMotionConfig(hold_missed_frames=5, output_smoothing_alpha=0.0)
    planner = SubMotionPlanner()
    tel = TelemetryContext.empty()
    moving = FakeOutput(apple_detected=True, drive_motor=0.8, error_x=0.3)
    plan_sub_motion(moving, telemetry=tel, cfg=cfg, planner=planner)

    centred = FakeOutput(apple_detected=True, drive_motor=0.0, error_x=0.0, error_y=0.0)
    r = plan_sub_motion(centred, telemetry=tel, cfg=cfg, planner=planner)
    assert r.actuators.thruster_x == 0.0


def test_yolo_linked_flap_couples_fins_to_aft_steer():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0, linked_flap=True)
    tel = TelemetryContext(roll=2.0, pitch=1.0)
    out = FakeOutput(
        apple_detected=True,
        steering_servo=0.6,
        camera_tilt_servo=-0.4,
        error_x=0.0,
        error_y=0.0,
    )
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.aft_steer_y > 0.4
    assert r.actuators.aft_steer_z < -0.3
    assert r.actuators.fin_left != 0.0 or r.actuators.fin_right != 0.0
    assert "linked_flap" in r.note
    assert abs(r.actuators.fin_left - r.actuators.fin_right) > 0.05


def test_yolo_linked_flap_off_uses_gyro_leveling():
    cfg = SubMotionConfig(output_smoothing_alpha=0.0, linked_flap=False)
    tel = TelemetryContext(roll=10.0, pitch=0.0)
    out = FakeOutput(apple_detected=True)
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.fin_left < 0
    assert r.actuators.fin_right > 0


def test_output_smoothing_accepts_snake_case_fields():
    cfg = SubMotionConfig(output_smoothing_alpha=0.5, hold_missed_frames=0)
    planner = SubMotionPlanner()
    tel = TelemetryContext.empty()
    a = plan_sub_motion(
        FakeOutput(apple_detected=True, steering_servo=1.0, error_x=0.8),
        telemetry=tel, cfg=cfg, planner=planner,
    )
    b = plan_sub_motion(
        FakeOutput(apple_detected=True, steering_servo=1.0, error_x=0.8),
        telemetry=tel, cfg=cfg, planner=planner,
    )
    assert a.actuators.aft_steer_y != 0.0
    assert abs(b.actuators.aft_steer_y) >= abs(a.actuators.aft_steer_y)
