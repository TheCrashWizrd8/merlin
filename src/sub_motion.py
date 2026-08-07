"""
sub_motion.py
-------------
Layered auto control: camera targets → attitude (fins) → aft steer → thruster,
with ballast trim when the apple is above/below the sub.

Control order (each frame):
  1. Fins — proportional roll/pitch leveling from gyro (realign body).
  2. Aft steer Y/Z — camera horizontal/vertical errors (scaled while still tilted).
  3. Thruster — forward only when attitude + alignment gates pass.
  4. Ballast — fill/drain when apple is below/above frame centre (error_y).

Telemetry safety (leak, battery) stays in controller.py for drive_motor;
this module handles actuators + ballast for the sub stack.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from src.sub_state import SubActuators
from src.telemetry_context import TelemetryContext

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"


@dataclass(frozen=True)
class SubMotionConfig:
    # Attitude leveling (fins)
    level_roll_gain: float = 0.035   # fin command per degree roll
    level_pitch_gain: float = 0.025  # fin command per degree pitch
    max_fin_command: float = 0.85
    max_roll_deg: float = 12.0       # full thruster allowed below this
    max_pitch_deg: float = 10.0

    # Gates
    require_level_for_steer: bool = True
    require_level_for_drive: bool = True
    max_error_x_for_drive: float = 0.40
    max_error_y_for_drive: float = 0.45

    # Ballast trim from vertical apple error (error_y: + = apple below centre)
    use_ballast_for_height: bool = True
    ballast_height_gain: float = 0.55
    ballast_error_deadzone: float = 0.12
    ballast_max_command: float = 1.0


@dataclass
class SubMotionResult:
    actuators: SubActuators
    ballast_fore: float = 0.0
    ballast_aft: float = 0.0
    phase: str = "idle"  # level | point | approach | idle
    note: str = ""


def _clamp(v: float, lo: float = -1.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, float(v)))


def load_sub_motion_config(path: Path = CONFIG_PATH) -> SubMotionConfig:
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    m = cfg.get("sub_motion") or {}
    approach = cfg.get("approach") or {}
    return SubMotionConfig(
        level_roll_gain=float(m.get("level_roll_gain", 0.035)),
        level_pitch_gain=float(m.get("level_pitch_gain", 0.025)),
        max_fin_command=float(m.get("max_fin_command", 0.85)),
        max_roll_deg=float(m.get("max_roll_deg", 12.0)),
        max_pitch_deg=float(m.get("max_pitch_deg", 10.0)),
        require_level_for_steer=bool(m.get("require_level_for_steer", True)),
        require_level_for_drive=bool(m.get("require_level_for_drive", True)),
        max_error_x_for_drive=float(
            m.get("max_error_x_for_drive", approach.get("max_error_x_for_drive", 0.40))
        ),
        max_error_y_for_drive=float(
            m.get("max_error_y_for_drive", approach.get("max_error_y_for_drive", 0.45))
        ),
        use_ballast_for_height=bool(m.get("use_ballast_for_height", True)),
        ballast_height_gain=float(m.get("ballast_height_gain", 0.55)),
        ballast_error_deadzone=float(m.get("ballast_error_deadzone", 0.12)),
        ballast_max_command=float(m.get("ballast_max_command", 1.0)),
    )


def _attitude_scale(roll_deg: float, pitch_deg: float, cfg: SubMotionConfig) -> float:
    """1.0 when level; ramps to 0 as roll/pitch exceed limits."""
    roll_ratio = abs(roll_deg) / max(cfg.max_roll_deg, 1e-3)
    pitch_ratio = abs(pitch_deg) / max(cfg.max_pitch_deg, 1e-3)
    worst = max(roll_ratio, pitch_ratio)
    if worst <= 1.0:
        return max(0.0, 1.0 - worst * 0.85)
    return max(0.0, 1.0 - worst)


def _alignment_scale(error_x: float, error_y: float, cfg: SubMotionConfig) -> float:
    sx = min(1.0, abs(error_x) / cfg.max_error_x_for_drive)
    sy = min(1.0, abs(error_y) / cfg.max_error_y_for_drive)
    misalign = max(sx, sy)
    return max(0.0, 1.0 - misalign)


def _fin_level_commands(roll_deg: float, pitch_deg: float, cfg: SubMotionConfig) -> tuple[float, float]:
    """
    Differential fore fins to counter roll; common-mode component for pitch.
    Matches Xbox fin mapping: positive fin_right − fin_left ≈ roll correction.
    """
    roll_cmd = _clamp(cfg.level_roll_gain * roll_deg, -cfg.max_fin_command, cfg.max_fin_command)
    pitch_cmd = _clamp(cfg.level_pitch_gain * pitch_deg, -cfg.max_fin_command, cfg.max_fin_command)
    fin_left = _clamp(-roll_cmd - pitch_cmd)
    fin_right = _clamp(roll_cmd - pitch_cmd)
    return fin_left, fin_right


def _ballast_for_height(error_y: float, cfg: SubMotionConfig, apple_detected: bool) -> tuple[float, float]:
    """
    error_y > 0 → apple below centre → sub should sink → fill (+).
    error_y < 0 → apple above centre → sub should rise → drain (−).
    """
    if not cfg.use_ballast_for_height or not apple_detected:
        return 0.0, 0.0
    if abs(error_y) <= cfg.ballast_error_deadzone:
        return 0.0, 0.0
    sign = 1.0 if error_y > 0 else -1.0
    mag = min(cfg.ballast_max_command, cfg.ballast_height_gain * abs(error_y))
    cmd = sign * mag
    return cmd, cmd


def plan_sub_motion(
    output: Any,
    telemetry: TelemetryContext | None = None,
    cfg: SubMotionConfig | None = None,
) -> SubMotionResult:
    """
    Map YOLO ControlOutput + gyro into sub actuators and ballast commands.

    Expects output with steering_servo, camera_tilt_servo, drive_motor,
    error_x, error_y, apple_detected.
    """
    if cfg is None:
        cfg = load_sub_motion_config()

    tel = telemetry or TelemetryContext.empty()
    roll = tel.roll if tel.roll is not None else 0.0
    pitch = tel.pitch if tel.pitch is not None else 0.0

    error_x = float(getattr(output, "error_x", 0.0))
    error_y = float(getattr(output, "error_y", 0.0))
    apple = bool(getattr(output, "apple_detected", False))

    steer_cmd = float(getattr(output, "steering_servo", 0.0))
    tilt_cmd = float(getattr(output, "camera_tilt_servo", 0.0))
    drive_cmd = float(getattr(output, "drive_motor", 0.0))

    att_scale = _attitude_scale(roll, pitch, cfg)
    align_scale = _alignment_scale(error_x, error_y, cfg) if apple else 0.0

    fin_l, fin_r = _fin_level_commands(roll, pitch, cfg)

    notes: list[str] = []
    if abs(roll) > 2.0 or abs(pitch) > 2.0:
        phase = "level"
        notes.append(f"roll={roll:.0f} pitch={pitch:.0f}")
    elif apple and align_scale < 0.95:
        phase = "point"
    elif apple and drive_cmd > 0.05:
        phase = "approach"
    else:
        phase = "idle"

    steer_scale = att_scale if cfg.require_level_for_steer else 1.0
    aft_y = _clamp(steer_cmd * steer_scale)
    aft_z = _clamp(tilt_cmd * steer_scale)

    thr_scale = 1.0
    if cfg.require_level_for_drive:
        thr_scale *= att_scale
    if apple:
        thr_scale *= align_scale
    thruster = _clamp(drive_cmd * thr_scale)

    if att_scale < 0.35:
        notes.append("leveling")
    if apple and align_scale < 0.5:
        notes.append("pointing")

    b_fore, b_aft = _ballast_for_height(error_y, cfg, apple)
    if b_fore != 0.0:
        notes.append("ballast_height")

    return SubMotionResult(
        actuators=SubActuators(
            aft_steer_y=aft_y,
            aft_steer_z=aft_z,
            thruster_x=thruster,
            fin_left=fin_l,
            fin_right=fin_r,
        ),
        ballast_fore=b_fore,
        ballast_aft=b_aft,
        phase=phase,
        note=",".join(notes),
    )
