"""Unit tests for layered sub motion + ballast height trim."""

from dataclasses import dataclass

from src.sub_motion import SubMotionConfig, plan_sub_motion
from src.telemetry_context import TelemetryContext


@dataclass
class FakeOutput:
    steering_servo: float = 0.0
    drive_motor: float = 0.0
    camera_tilt_servo: float = 0.0
    error_x: float = 0.0
    error_y: float = 0.0
    apple_detected: bool = False


def test_fins_counter_roll():
    cfg = SubMotionConfig()
    tel = TelemetryContext(roll=10.0, pitch=0.0)
    out = FakeOutput(apple_detected=True)
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.fin_left < 0
    assert r.actuators.fin_right > 0
    assert r.phase == "level"


def test_thruster_gated_when_rolled():
    cfg = SubMotionConfig()
    tel = TelemetryContext(roll=20.0, pitch=0.0)
    out = FakeOutput(drive_motor=1.0, apple_detected=True, error_x=0.0, error_y=0.0)
    r = plan_sub_motion(out, telemetry=tel, cfg=cfg)
    assert r.actuators.thruster_x < 0.5


def test_ballast_apple_below_fills():
    cfg = SubMotionConfig()
    out = FakeOutput(apple_detected=True, error_y=0.5)
    r = plan_sub_motion(out, telemetry=TelemetryContext.empty(), cfg=cfg)
    assert r.ballast_fore > 0
    assert r.ballast_aft > 0


def test_ballast_apple_above_drains():
    cfg = SubMotionConfig()
    out = FakeOutput(apple_detected=True, error_y=-0.5)
    r = plan_sub_motion(out, telemetry=TelemetryContext.empty(), cfg=cfg)
    assert r.ballast_fore < 0
    assert r.ballast_aft < 0


def test_ballast_deadzone():
    cfg = SubMotionConfig()
    out = FakeOutput(apple_detected=True, error_y=0.05)
    r = plan_sub_motion(out, telemetry=TelemetryContext.empty(), cfg=cfg)
    assert r.ballast_fore == 0.0
    assert r.ballast_aft == 0.0
