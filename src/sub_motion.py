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
from dataclasses import asdict, dataclass, replace
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

    # Dropout hold + output smoothing (prevents frame-by-frame stop on missed detections)
    hold_missed_frames: int = 12
    output_smoothing_alpha: float = 0.40  # EMA on steer/thruster/ballast; 0 = off
    linked_flap: bool = True  # fins oppose aft steer (Xbox linked-flap behaviour)


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
    ctrl = cfg.get("hold_missed_frames")
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
        hold_missed_frames=int(m.get("hold_missed_frames", ctrl if ctrl is not None else 12)),
        output_smoothing_alpha=float(m.get("output_smoothing_alpha", 0.40)),
        linked_flap=bool(m.get("linked_flap", True)),
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


def _linked_flap_active(cfg: SubMotionConfig) -> bool:
    if not cfg.linked_flap:
        return False
    try:
        from src.sub_state import get_sub_state

        return get_sub_state().is_linked_flap_enabled()
    except Exception:
        return True


def _fins_with_linked_flap(
    fin_left: float,
    fin_right: float,
    aft_y: float,
    aft_z: float,
    cfg: SubMotionConfig,
) -> tuple[float, float]:
    if not _linked_flap_active(cfg):
        return fin_left, fin_right
    from src.xbox_mapping import apply_linked_flap_to_fins, linked_flap_replace_stick

    replace = linked_flap_replace_stick()
    base_l = 0.0 if replace else fin_left
    base_r = 0.0 if replace else fin_right
    return apply_linked_flap_to_fins(
        base_l,
        base_r,
        aft_y,
        aft_z,
        enabled=True,
    )


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


def _compute_sub_motion(
    output: Any,
    telemetry: TelemetryContext,
    cfg: SubMotionConfig,
) -> SubMotionResult:
    roll = telemetry.roll if telemetry.roll is not None else 0.0
    pitch = telemetry.pitch if telemetry.pitch is not None else 0.0

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

    fin_l, fin_r = _fins_with_linked_flap(fin_l, fin_r, aft_y, aft_z, cfg)
    if _linked_flap_active(cfg):
        notes.append("linked_flap")

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


def _is_camera_motion_active(result: SubMotionResult, threshold: float = 0.04) -> bool:
    a = result.actuators
    if abs(a.thruster_x) > threshold:
        return True
    if abs(a.aft_steer_y) > threshold or abs(a.aft_steer_z) > threshold:
        return True
    if abs(result.ballast_fore) > threshold or abs(result.ballast_aft) > threshold:
        return True
    return False


def _merge_motion_hold(
    live: SubMotionResult,
    held: SubMotionResult,
    cfg: SubMotionConfig,
) -> SubMotionResult:
    """Hold camera-driven outputs; fins follow linked aft steer or live gyro."""
    note = held.note + ",hold" if held.note else "hold"
    if _linked_flap_active(cfg):
        fin_l, fin_r = _fins_with_linked_flap(
            0.0,
            0.0,
            held.actuators.aft_steer_y,
            held.actuators.aft_steer_z,
            cfg,
        )
    else:
        fin_l = live.actuators.fin_left
        fin_r = live.actuators.fin_right
    return SubMotionResult(
        actuators=SubActuators(
            aft_steer_y=held.actuators.aft_steer_y,
            aft_steer_z=held.actuators.aft_steer_z,
            thruster_x=held.actuators.thruster_x,
            fin_left=fin_l,
            fin_right=fin_r,
        ),
        ballast_fore=held.ballast_fore,
        ballast_aft=held.ballast_aft,
        phase=held.phase,
        note=note,
    )


def _smooth_actuators(prev: SubActuators | None, new: SubActuators, alpha: float) -> SubActuators:
    if prev is None or alpha <= 0.0:
        return new
    if alpha >= 1.0:
        return new
    p = asdict(prev)
    n = asdict(new)
    blended = {k: alpha * n[k] + (1.0 - alpha) * p[k] for k in p}
    return SubActuators(**blended)


class SubMotionPlanner:
    """Stateful planner: holds last motion through dropouts and EMA-smooths outputs."""

    def __init__(self) -> None:
        self._last_motion: SubMotionResult | None = None
        self._smoothed: SubActuators | None = None
        self._missed_frames = 0

    def reset(self) -> None:
        self._last_motion = None
        self._smoothed = None
        self._missed_frames = 0

    def plan(
        self,
        output: Any,
        telemetry: TelemetryContext | None = None,
        cfg: SubMotionConfig | None = None,
    ) -> SubMotionResult:
        if cfg is None:
            cfg = load_sub_motion_config()

        tel = telemetry or TelemetryContext.empty()
        live = _compute_sub_motion(output, tel, cfg)
        apple = bool(getattr(output, "apple_detected", False))

        if apple and _is_camera_motion_active(live):
            self._last_motion = live
            self._missed_frames = 0
            result = live
        elif apple:
            # Target visible and centred — intentional stop, don't latch old thrust.
            self._last_motion = None
            self._missed_frames = 0
            result = live
        elif self._last_motion is not None and self._missed_frames < cfg.hold_missed_frames:
            self._missed_frames += 1
            result = _merge_motion_hold(live, self._last_motion, cfg)
        else:
            self._missed_frames += 1
            if self._missed_frames > cfg.hold_missed_frames:
                self._last_motion = None
            result = live

        if cfg.output_smoothing_alpha > 0.0:
            smoothed = _smooth_actuators(
                self._smoothed, result.actuators, cfg.output_smoothing_alpha
            )
            self._smoothed = smoothed
            result = replace(result, actuators=smoothed)

        return result


_default_planner = SubMotionPlanner()


def reset_sub_motion_hold() -> None:
    """Clear hold/smoothing state (for tests)."""
    _default_planner.reset()


def plan_sub_motion(
    output: Any,
    telemetry: TelemetryContext | None = None,
    cfg: SubMotionConfig | None = None,
    planner: SubMotionPlanner | None = None,
) -> SubMotionResult:
    """
    Map YOLO ControlOutput + gyro into sub actuators and ballast commands.

    Expects output with steering_servo, camera_tilt_servo, drive_motor,
    error_x, error_y, apple_detected.
    """
    p = planner or _default_planner
    return p.plan(output, telemetry=telemetry, cfg=cfg)
