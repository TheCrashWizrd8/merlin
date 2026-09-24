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
    raw_buttons = cfg.get("buttons") or {}
    for key, val in raw_buttons.items():
        if isinstance(val, int):
            buttons[key] = val
        elif isinstance(val, str) and val.isdigit():
            buttons[key] = int(val)
        else:
            buttons[key] = val
    return {
        "sticks": cfg.get("sticks") or {},
        "buttons": buttons,
        "triggers": cfg.get("triggers") or {},
        "ballast": cfg.get("ballast") or {},
        "thruster": cfg.get("thruster") or {},
        "flaps": cfg.get("flaps") or {},
        "modes": cfg.get("modes") or {},
        "leak": cfg.get("leak") or {},
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


def _trigger(
    xbox: XboxState,
    name: str,
    indices: dict[str, Any],
    *,
    trigger_deadzone: float = 0.0,
) -> float:
    raw = 0.0
    if name in xbox.triggers:
        raw = float(xbox.triggers[name])
    elif name in xbox.buttons and xbox.buttons[name]:
        raw = 1.0
    else:
        idx = indices.get(name)
        if idx is not None and xbox.buttons.get(str(idx), xbox.buttons.get(name, False)):
            raw = 1.0
    if raw <= 0.0:
        return 0.0
    if trigger_deadzone > 0.0 and raw < trigger_deadzone:
        return 0.0
    if trigger_deadzone < 1.0:
        raw = (raw - trigger_deadzone) / (1.0 - trigger_deadzone)
    return max(0.0, min(1.0, raw))


def _btn(xbox: XboxState, name: str, indices: dict[str, Any]) -> bool:
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


def _stick_component(x: float, y: float, axis: str) -> float:
    axis = (axis or "x").lower()
    if axis == "y":
        return y
    if axis == "x":
        return x
    return 0.0


def _clamp(v: float) -> float:
    return max(-1.0, min(1.0, float(v)))


# Bearing from up (+clockwise °) → (fin_left, fin_right)
_FIN_KEYPOINTS: list[tuple[float, float, float]] = [
    (0.0, 1.0, 1.0),
    (45.0, 0.0, 1.0),
    (-45.0, 1.0, 0.0),
    (90.0, 0.0, 0.0),
    (-90.0, 0.0, 0.0),
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

    sorted_kp = sorted(_FIN_KEYPOINTS, key=lambda k: k[0])
    extended: list[tuple[float, float, float]] = [
        (-180.0, -1.0, -1.0),
        *sorted_kp,
        (180.0, -1.0, -1.0),
    ]

    fl = fr = 0.0
    for i in range(len(extended) - 1):
        b0, l0, r0 = extended[i]
        b1, l1, r1 = extended[i + 1]
        if b0 <= bearing <= b1:
            t = (bearing - b0) / (b1 - b0) if b1 != b0 else 0.0
            fl = l0 + t * (l1 - l0)
            fr = r0 + t * (r1 - r0)
            break

    return _clamp(mag * fl), _clamp(mag * fr)


def _map_flaps_stick(
    rs_x: float,
    rs_y: float,
    deadzone: float,
    flap_cfg: dict[str, Any],
) -> tuple[float, float]:
    """
    Right-stick flap mode:
      up/down → both fins same direction
      left/right → differential roll
    """
    invert_y = bool(flap_cfg.get("invert_y", False))
    invert_x = bool(flap_cfg.get("invert_x", False))
    if invert_y:
        rs_y = -rs_y
    if invert_x:
        rs_x = -rs_x

    rs_x, rs_y = _apply_circular_deadzone(rs_x, rs_y, deadzone)
    ax, ay = abs(rs_x), abs(rs_y)
    if ax < 0.05 and ay < 0.05:
        return 0.0, 0.0

    if ay >= ax:
        mag = min(1.0, ay)
        if rs_y < 0.0:
            return mag, mag
        return -mag, -mag

    mag = min(1.0, ax)
    if rs_x < 0.0:
        return mag, -mag
    return -mag, mag


def _map_thruster(
    xbox: XboxState,
    thr_cfg: dict[str, Any],
    btn_idx: dict[str, Any],
    *,
    trigger_deadzone: float = 0.08,
) -> float:
    fwd_key = thr_cfg.get("forward_trigger", thr_cfg.get("forward_button", "rt"))
    rev_key = thr_cfg.get("reverse_trigger", thr_cfg.get("reverse_button", "lt"))
    fwd_mag = float(thr_cfg.get("forward_magnitude", 1.0))
    rev_mag = float(thr_cfg.get("reverse_magnitude", 1.0))

    if fwd_key in ("lt", "rt"):
        fwd = _trigger(xbox, fwd_key, btn_idx, trigger_deadzone=trigger_deadzone) * fwd_mag
    else:
        fwd = fwd_mag if _btn(xbox, fwd_key, btn_idx) else 0.0

    if rev_key in ("lt", "rt"):
        rev = _trigger(xbox, rev_key, btn_idx, trigger_deadzone=trigger_deadzone) * rev_mag
    else:
        rev = rev_mag if _btn(xbox, rev_key, btn_idx) else 0.0

    if fwd > 0.05 and fwd >= rev:
        return _clamp(fwd)
    if rev > 0.05:
        return _clamp(-rev)
    return 0.0


def apply_linked_flap_to_fins(
    fin_left: float,
    fin_right: float,
    aft_steer_y: float,
    aft_steer_z: float,
    *,
    enabled: bool,
    config: dict[str, Any] | None = None,
) -> tuple[float, float]:
    """Couple fore fins to aft steer (shared by Xbox mapping and YOLO auto)."""
    cfg = config or load_mapping_config()
    linked_cfg = (cfg.get("modes") or {}).get("linked_flap") or {}
    return _apply_linked_flap(
        fin_left,
        fin_right,
        aft_steer_y,
        aft_steer_z,
        linked_cfg,
        enabled=enabled,
    )


def linked_flap_replace_stick(config: dict[str, Any] | None = None) -> bool:
    cfg = config or load_mapping_config()
    linked = (cfg.get("modes") or {}).get("linked_flap") or {}
    return bool(linked.get("replace_stick", True))


def _apply_linked_flap(
    fin_left: float,
    fin_right: float,
    aft_y: float,
    aft_z: float,
    linked_cfg: dict[str, Any],
    *,
    enabled: bool,
) -> tuple[float, float]:
    """
    Linked flap: fore fins oppose aft steer (left stick), independent of thrust.
    aft_steer_z → common-mode fin deflection; aft_steer_y → differential roll.
    """
    if not enabled:
        return fin_left, fin_right

    gain = float(linked_cfg.get("gain", 1.0))
    angle_dz = float(linked_cfg.get("angle_deadzone", 0.05))
    ay = 0.0 if abs(aft_y) < angle_dz else aft_y
    az = 0.0 if abs(aft_z) < angle_dz else aft_z
    if ay == 0.0 and az == 0.0:
        return 0.0, 0.0

    # Match aft/fin servo sign conventions: aft up is −Z, fin up is +; same numeric
    # sign on both axes yields opposite physical deflection (compensating).
    common = az * gain
    diff = ay * gain
    coupled_l = common + diff
    coupled_r = common - diff

    if bool(linked_cfg.get("replace_stick", True)):
        return _clamp(coupled_l), _clamp(coupled_r)
    return _clamp(fin_left + coupled_l), _clamp(fin_right + coupled_r)


def linked_flap_default_enabled(config: dict[str, Any] | None = None) -> bool:
    cfg = config or load_mapping_config()
    linked = (cfg.get("modes") or {}).get("linked_flap") or {}
    return bool(linked.get("default_on", True))


def linked_flap_button_name(config: dict[str, Any] | None = None) -> str:
    cfg = config or load_mapping_config()
    linked = (cfg.get("modes") or {}).get("linked_flap") or {}
    return str(linked.get("button", "dpad_left"))


def emergency_stop_button_name(config: dict[str, Any] | None = None) -> str:
    cfg = config or load_mapping_config()
    estop = cfg.get("emergency_stop") or {}
    if estop.get("enabled") is False:
        return ""
    return str(estop.get("button", "b"))


def check_emergency_stop(
    xbox: XboxState,
    *,
    button_was_pressed: bool,
    config: dict[str, Any] | None = None,
) -> tuple[bool, bool]:
    """
    Edge-detect emergency-stop button press.
    Returns (triggered_this_frame, button_is_pressed).
    """
    btn = emergency_stop_button_name(config)
    if not btn:
        return False, False
    cfg = config or load_mapping_config()
    pressed = _btn(xbox, btn, cfg["buttons"])
    return pressed and not button_was_pressed, pressed


def update_linked_flap_toggle(
    xbox: XboxState,
    *,
    enabled: bool,
    button_was_pressed: bool,
    config: dict[str, Any] | None = None,
) -> tuple[bool, bool]:
    """
    Edge-detect linked-flap toggle button.
    Returns (new_enabled, button_is_pressed).
    """
    cfg = config or load_mapping_config()
    linked = (cfg.get("modes") or {}).get("linked_flap") or {}
    btn = str(linked.get("button", "dpad_left"))
    toggle = bool(linked.get("toggle", True))
    btn_idx = cfg["buttons"]
    pressed = _btn(xbox, btn, btn_idx)

    if toggle and pressed and not button_was_pressed:
        enabled = not enabled
    return enabled, pressed


def map_xbox_to_actuators(
    xbox: XboxState,
    deadzone: float = 0.12,
    config: dict[str, Any] | None = None,
    *,
    linked_flap_enabled: bool | None = None,
    trigger_deadzone: float = 0.08,
) -> SubActuators:
    cfg = config or load_mapping_config()
    btn_idx: dict[str, Any] = cfg["buttons"]
    left = cfg["sticks"].get("left") or {}
    right = cfg["sticks"].get("right") or {}
    thr_cfg = cfg["thruster"] or {}
    flap_cfg = cfg["flaps"] or {}
    linked_cfg = (cfg.get("modes") or {}).get("linked_flap") or {}
    left_dz = _stick_deadzone(left, deadzone)
    right_dz = _stick_deadzone(right, deadzone)

    ls_x, ls_y = _apply_circular_deadzone(xbox.left_stick_x, xbox.left_stick_y, left_dz)
    if left.get("invert_y", False):
        ls_y = -ls_y
    if left.get("invert_x", False):
        ls_x = -ls_x

    aft_y = _stick_component(ls_x, ls_y, left.get("aft_steer_x", left.get("aft_steer_y", "x")))
    aft_z = _stick_component(ls_x, ls_y, left.get("aft_steer_y", left.get("aft_steer_z", "y")))

    thruster = _map_thruster(
        xbox, thr_cfg, btn_idx, trigger_deadzone=trigger_deadzone
    )

    if linked_flap_enabled is None:
        linked_flap_enabled = linked_flap_default_enabled(cfg)

    rs_x, rs_y = xbox.right_stick_x, xbox.right_stick_y
    if linked_flap_enabled and bool(linked_cfg.get("replace_stick", True)):
        fin_left, fin_right = 0.0, 0.0
    else:
        right_mode = (right.get("mode") or flap_cfg.get("mode") or "flaps").lower()
        if right_mode in ("flaps", "normal"):
            fin_left, fin_right = _map_flaps_stick(rs_x, rs_y, right_dz, flap_cfg)
        elif right_mode == "polar_fins":
            snap = float(right.get("snap_degrees", 5.0))
            fin_left, fin_right = _map_polar_fins(rs_x, rs_y, right_dz, snap_degrees=snap)
        else:
            rs_x, rs_y = _apply_circular_deadzone(rs_x, rs_y, right_dz)
            fin_left = _clamp(-rs_x)
            fin_right = _clamp(rs_x)

    fin_left, fin_right = _apply_linked_flap(
        fin_left,
        fin_right,
        aft_y,
        aft_z,
        linked_cfg,
        enabled=linked_flap_enabled,
    )

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
    btn_idx: dict[str, Any] = cfg["buttons"]
    ballast = cfg["ballast"] or {}

    up_btn = ballast.get("up_button", ballast.get("fill_button", "rb"))
    down_btn = ballast.get("down_button", ballast.get("drain_button", "lb"))
    up_mag = float(ballast.get("up_magnitude", ballast.get("fill_magnitude", 1.0)))
    down_mag = float(ballast.get("down_magnitude", ballast.get("drain_magnitude", 1.0)))

    up = _btn(xbox, up_btn, btn_idx)
    down = _btn(xbox, down_btn, btn_idx)

    if up and down:
        return 0.0, 0.0
    if not up and not down:
        legacy_fill = ballast.get("fill_trigger")
        if legacy_fill:
            fill_amount = _trigger(
                xbox, legacy_fill, btn_idx, trigger_deadzone=trigger_deadzone
            )
            if fill_amount > 0.05:
                cmd = fill_amount * up_mag
            elif _btn(xbox, down_btn, btn_idx):
                cmd = -down_mag
            else:
                return 0.0, 0.0
        else:
            return 0.0, 0.0
    else:
        cmd = up_mag if up else -down_mag

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


def xbox_input_active(
    xbox: XboxState,
    actuators: SubActuators,
    *,
    config: dict[str, Any] | None = None,
    trigger_deadzone: float = 0.08,
    actuator_threshold: float = 0.12,
) -> bool:
    """
    True when the operator deliberately uses the pad (auto-mode override).

    Pass filtered sticks and actuators from map_xbox_to_actuators (after deadzone).
    Raw stick magnitude is ignored so drift does not steal control from YOLO.
    """
    cfg = config or load_mapping_config()
    btn_idx: dict[str, Any] = cfg["buttons"]

    for v in xbox.triggers.values():
        if abs(float(v)) >= trigger_deadzone:
            return True

    fore, aft = map_xbox_ballast(xbox, cfg, trigger_deadzone=trigger_deadzone)
    if fore != 0.0 or aft != 0.0:
        return True

    thr_cfg = cfg.get("thruster") or {}
    ballast = cfg.get("ballast") or {}
    estop_btn = emergency_stop_button_name(cfg)
    for btn in (
        thr_cfg.get("forward_trigger", thr_cfg.get("forward_button", "a")),
        thr_cfg.get("reverse_trigger", thr_cfg.get("reverse_button", "b")),
        ballast.get("up_button", ballast.get("drain_button", "lb")),
        ballast.get("down_button", ballast.get("drain_button", "lb")),
        linked_flap_button_name(cfg),
    ):
        if btn in ("lt", "rt") or (estop_btn and btn == estop_btn):
            continue
        if _btn(xbox, btn, btn_idx):
            return True

    thr = float(actuators.thruster_x)
    if abs(thr) >= actuator_threshold:
        return True

    for v in (
        actuators.aft_steer_y,
        actuators.aft_steer_z,
        actuators.fin_left,
        actuators.fin_right,
    ):
        if abs(float(v)) >= actuator_threshold:
            return True

    return False
