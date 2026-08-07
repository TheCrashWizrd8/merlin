"""
xbox_mapping.py
---------------
Load config/xbox_mapping.yaml and map XboxState → actuators + ballast.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from src.sub_state import SubActuators, XboxState

DEFAULT_MAPPING_PATH = Path(__file__).parent.parent / "config" / "xbox_mapping.yaml"
HARDWARE_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"

# SDL joystick button names → default index on Linux Xbox pads
_DEFAULT_BUTTONS = {"a": 0, "b": 1, "x": 2, "y": 3, "lb": 4, "rb": 5}


def _load_yaml(path: Path) -> dict[str, Any]:
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def mapping_path() -> Path:
    hw = _load_yaml(HARDWARE_PATH)
    xbox = hw.get("xbox") or {}
    rel = xbox.get("mapping_file")
    if rel:
        return Path(__file__).parent.parent / rel
    return DEFAULT_MAPPING_PATH


def load_mapping_config(path: Path | None = None) -> dict[str, Any]:
    cfg = _load_yaml(path or mapping_path())
    buttons = dict(_DEFAULT_BUTTONS)
    buttons.update(cfg.get("buttons") or {})
    return {
        "sticks": cfg.get("sticks") or {},
        "buttons": buttons,
        "triggers": cfg.get("triggers") or {},
        "ballast": cfg.get("ballast") or {},
        "thruster": cfg.get("thruster") or {},
    }


def _apply_deadzone(v: float, dz: float) -> float:
    if abs(v) < dz:
        return 0.0
    sign = 1.0 if v > 0 else -1.0
    return sign * (abs(v) - dz) / (1.0 - dz)


def _apply_circular_deadzone(x: float, y: float, dz: float) -> tuple[float, float]:
    """Zero small stick deflection — fixes drift better than per-axis deadzone."""
    if dz <= 0.0:
        return x, y
    mag = math.hypot(x, y)
    if mag < dz:
        return 0.0, 0.0
    scale = (mag - dz) / (mag * (1.0 - dz))
    return x * scale, y * scale


def _stick_deadzone(stick_cfg: dict[str, Any], default: float) -> float:
    if "deadzone" in stick_cfg:
        return float(stick_cfg["deadzone"])
    return default


def _trigger(xbox: XboxState, name: str, indices: dict[str, int]) -> float:
    if name in xbox.triggers:
        return float(xbox.triggers[name])
    if name in xbox.buttons and xbox.buttons[name]:
        return 1.0
    idx = indices.get(name)
    if idx is not None and xbox.buttons.get(str(idx), xbox.buttons.get(name, False)):
        return 1.0
    return 0.0


def _btn(xbox: XboxState, name: str, indices: dict[str, int]) -> bool:
    if name in xbox.buttons:
        return bool(xbox.buttons[name])
    idx = indices.get(name)
    if idx is None:
        return False
    return bool(xbox.buttons.get(str(idx), xbox.buttons.get(name, False)))


def _dpad(xbox: XboxState) -> tuple[bool, bool, bool, bool]:
    up = bool(xbox.buttons.get("dpad_up"))
    down = bool(xbox.buttons.get("dpad_down"))
    left = bool(xbox.buttons.get("dpad_left"))
    right = bool(xbox.buttons.get("dpad_right"))
    return up, down, left, right


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed distance between bearings in degrees."""
    return (a - b + 180.0) % 360.0 - 180.0


# Bearing from up (+clockwise °) → (fin_left, fin_right)
_FIN_KEYPOINTS: list[tuple[float, float, float]] = [
    (0.0, 1.0, 1.0),
    (45.0, 0.0, 1.0),
    (-45.0, 1.0, 0.0),
    (135.0, 0.0, -1.0),
    (-135.0, -1.0, 0.0),
    (180.0, -1.0, -1.0),
]


def _map_polar_fins(
    rs_x: float,
    rs_y: float,
    deadzone: float,
    *,
    snap_degrees: float = 5.0,
) -> tuple[float, float]:
    rs_x, rs_y = _apply_circular_deadzone(rs_x, rs_y, deadzone)
    mag = min(1.0, math.hypot(rs_x, rs_y))
    if mag <= 0.0:
        return 0.0, 0.0

    bearing = math.degrees(math.atan2(rs_x, -rs_y))
    half = max(0.0, float(snap_degrees))

    for target, fl, fr in _FIN_KEYPOINTS:
        if abs(_angle_diff(bearing, target)) <= half:
            return mag * fl, mag * fr
    if abs(abs(bearing) - 180.0) <= half:
        return -mag, -mag

    # Between snap zones: both fins equal (up/down from vertical stick component)
    both = max(-1.0, min(1.0, mag * math.cos(math.radians(bearing))))
    return both, both


def map_xbox_to_actuators(
    xbox: XboxState,
    deadzone: float = 0.12,
    config: dict[str, Any] | None = None,
) -> SubActuators:
    cfg = config or load_mapping_config()
    btn_idx: dict[str, int] = cfg["buttons"]
    left = cfg["sticks"].get("left") or {}
    right = cfg["sticks"].get("right") or {}
    thr_cfg = cfg["thruster"] or {}
    left_dz = _stick_deadzone(left, deadzone)
    right_dz = _stick_deadzone(right, deadzone)

    ls_x, ls_y = _apply_circular_deadzone(xbox.left_stick_x, xbox.left_stick_y, left_dz)
    if left.get("invert_y", True):
        ls_y = -ls_y

    aft_y = ls_x if left.get("aft_steer_y", "x") == "x" else ls_y
    aft_z = ls_y if left.get("aft_steer_z", "y") == "y" else ls_x

    rs_x, rs_y = xbox.right_stick_x, xbox.right_stick_y
    snap = float(right.get("snap_degrees", 5.0))
    if (right.get("mode") or "polar_fins") == "polar_fins":
        fin_left, fin_right = _map_polar_fins(rs_x, rs_y, right_dz, snap_degrees=snap)
    else:
        rs_x, rs_y = _apply_circular_deadzone(rs_x, rs_y, right_dz)
        fin_left = max(-1.0, min(1.0, -rs_x))
        fin_right = max(-1.0, min(1.0, rs_x))

    thruster = 0.0
    fwd_btn = thr_cfg.get("forward_button", "a")
    rev_btn = thr_cfg.get("reverse_button", "b")
    fwd_mag = float(thr_cfg.get("forward_magnitude", 1.0))
    rev_mag = float(thr_cfg.get("reverse_magnitude", 1.0))
    fwd = _trigger(xbox, fwd_btn, btn_idx) if fwd_btn in ("lt", "rt") else (
        fwd_mag if _btn(xbox, fwd_btn, btn_idx) else 0.0
    )
    rev = _trigger(xbox, rev_btn, btn_idx) if rev_btn in ("lt", "rt") else (
        rev_mag if _btn(xbox, rev_btn, btn_idx) else 0.0
    )
    if fwd > 0.05 and fwd >= rev:
        thruster = fwd
    elif rev > 0.05:
        thruster = -rev

    return SubActuators(
        aft_steer_y=aft_y,
        aft_steer_z=aft_z,
        thruster_x=thruster,
        fin_left=fin_left,
        fin_right=fin_right,
    )


def map_xbox_ballast(
    xbox: XboxState,
    config: dict[str, Any] | None = None,
    trigger_deadzone: float = 0.08,
) -> tuple[float, float]:
    """
    Return (fore, aft) ballast commands in -1..+1.
    Negative = drain, positive = fill, 0 = stop.
    """
    cfg = config or load_mapping_config()
    btn_idx: dict[str, int] = cfg["buttons"]
    ballast = cfg["ballast"] or {}

    drain_btn = ballast.get("drain_button", "lb")
    fill_trg = ballast.get("fill_trigger", "lt")
    drain_mag = float(ballast.get("drain_magnitude", 1.0))

    draining = _btn(xbox, drain_btn, btn_idx)
    fill_amount = max(0.0, min(1.0, float(xbox.triggers.get(fill_trg, 0.0))))
    if fill_amount < trigger_deadzone:
        fill_amount = 0.0
    elif trigger_deadzone < 1.0:
        fill_amount = (fill_amount - trigger_deadzone) / (1.0 - trigger_deadzone)
    filling = fill_amount > 0.05

    if draining and filling:
        return 0.0, 0.0
    if not draining and not filling:
        return 0.0, 0.0

    cmd = fill_amount if filling else -drain_mag

    dpad_up, dpad_down, _, _ = _dpad(xbox)
    if dpad_up and not dpad_down:
        tank = ballast.get("dpad_up_tank", "fore")
    elif dpad_down and not dpad_up:
        tank = ballast.get("dpad_down_tank", "aft")
    else:
        tank = ballast.get("default_tank", "both")

    if tank == "fore":
        return cmd, 0.0
    if tank == "aft":
        return 0.0, cmd
    return cmd, cmd
