"""
servo_center.py
---------------
Per-channel PCA9685 neutral ticks (command 0) for fins on channels 2–5.
Stored in config/pins.yaml; pushed to ESP32 on connect via CAL S <ch> <tick>.
"""

from __future__ import annotations

from pathlib import Path

import yaml

PINS_PATH = Path(__file__).parent.parent / "config" / "pins.yaml"
SERVO_MIN_TICK = 150
SERVO_MAX_TICK = 600
DEFAULT_CENTER_TICK = 391

# sub_rc.ino: ch2=aftZ, ch3=aftY, ch4=finL, ch5=finR
FIN_CHANNELS: tuple[int, ...] = (2, 3, 4, 5)
CHANNEL_LABELS: dict[int, str] = {
    2: "Aft Z",
    3: "Aft Y",
    4: "Fin L",
    5: "Fin R",
}


def clamp_tick(tick: int) -> int:
    return max(SERVO_MIN_TICK, min(SERVO_MAX_TICK, int(tick)))


def deg_to_tick(deg: float) -> int:
    d = max(0.0, min(180.0, float(deg)))
    span = float(SERVO_MAX_TICK - SERVO_MIN_TICK)
    return int(SERVO_MIN_TICK + (d / 180.0) * span + 0.5)


def tick_to_deg(tick: int) -> float:
    t = clamp_tick(tick)
    span = float(SERVO_MAX_TICK - SERVO_MIN_TICK)
    return (t - SERVO_MIN_TICK) / span * 180.0


def _default_tick_from_legacy(data: dict) -> int:
    cal = (data.get("pca9685") or {}).get("servo_tick_calibration") or {}
    if cal.get("centre_tick") is not None:
        return clamp_tick(int(cal["centre_tick"]))
    if cal.get("centre_deg") is not None:
        return deg_to_tick(float(cal["centre_deg"]))
    return DEFAULT_CENTER_TICK


def tick_delta_for_command(
    v: float,
    center: int,
    *,
    min_tick: int | None = None,
    max_tick: int | None = None,
) -> float:
    """Signed PWM tick offset from neutral for normalized command v in [-1, 1]."""
    v = max(-1.0, min(1.0, float(v)))
    t_min = clamp_tick(min_tick if min_tick is not None else SERVO_MIN_TICK)
    t_max = clamp_tick(max_tick if max_tick is not None else SERVO_MAX_TICK)
    center = clamp_tick(center)
    t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
    if v >= 0.0:
        return v * float(t_max - center)
    return v * float(center - t_min)


def command_for_tick_delta(
    delta: float,
    center: int,
    *,
    min_tick: int | None = None,
    max_tick: int | None = None,
) -> float:
    """Inverse of tick_delta_for_command (clamped to [-1, 1])."""
    t_min = clamp_tick(min_tick if min_tick is not None else SERVO_MIN_TICK)
    t_max = clamp_tick(max_tick if max_tick is not None else SERVO_MAX_TICK)
    center = clamp_tick(center)
    t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
    if delta >= 0.0:
        span = float(t_max - center)
        if span <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, delta / span))
    span = float(center - t_min)
    if span <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, delta / span))


def fin_right_inverted_matched_command(
    fin_left_command: float,
    *,
    left_center: int,
    right_center: int,
    left_min: int | None = None,
    left_max: int | None = None,
    right_min: int | None = None,
    right_max: int | None = None,
) -> float:
    """
    Fin R mounted inverted: opposite physical deflection from fin L with
    the same tick travel (per-channel neutral and min/max from sub_rc).
    """
    d_left = tick_delta_for_command(
        fin_left_command,
        left_center,
        min_tick=left_min,
        max_tick=left_max,
    )
    return command_for_tick_delta(
        -d_left,
        right_center,
        min_tick=right_min,
        max_tick=right_max,
    )


def _default_travel_pair(data: dict) -> tuple[int, int]:
    cal = (data.get("pca9685") or {}).get("servo_tick_calibration") or {}
    t_min = int(cal.get("min_tick", SERVO_MIN_TICK))
    t_max = int(cal.get("max_tick", SERVO_MAX_TICK))
    return clamp_tick(t_min), clamp_tick(t_max)


def _sanitize_travel_ticks(center: int, t_min: int, t_max: int) -> tuple[int, int]:
    """Ensure min < centre < max (PCA ticks). Recover from min=max=centre cal mistakes."""
    center = clamp_tick(center)
    lo = clamp_tick(t_min)
    hi = clamp_tick(t_max)
    if lo >= center:
        lo = SERVO_MIN_TICK
    if hi <= center:
        hi = SERVO_MAX_TICK
    lo = min(lo, center - 1) if center > SERVO_MIN_TICK else lo
    hi = max(hi, center + 1) if center < SERVO_MAX_TICK else hi
    if lo >= center:
        lo = max(SERVO_MIN_TICK, center - 1)
    if hi <= center:
        hi = min(SERVO_MAX_TICK, center + 1)
    if lo >= hi:
        lo = max(SERVO_MIN_TICK, min(center - 1, center))
        hi = min(SERVO_MAX_TICK, max(center + 1, center))
    return lo, hi


def read_servo_travel_ticks() -> dict[int, tuple[int, int]]:
    try:
        with open(PINS_PATH) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        data = {}
    default_min, default_max = _default_travel_pair(data)
    raw = (data.get("pca9685") or {}).get("servo_travel_ticks") or {}
    centers = read_servo_center_ticks()
    out: dict[int, tuple[int, int]] = {}
    for ch in FIN_CHANNELS:
        entry = raw.get(ch, raw.get(str(ch))) or {}
        if isinstance(entry, dict):
            t_min = clamp_tick(int(entry.get("min", default_min)))
            t_max = clamp_tick(int(entry.get("max", default_max)))
        else:
            t_min, t_max = default_min, default_max
        center = centers[ch]
        t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
        out[ch] = (t_min, t_max)
    return out


def write_servo_travel_tick(channel: int, end: str, tick: int) -> dict[str, int | str]:
    ch = int(channel)
    if ch not in FIN_CHANNELS:
        raise ValueError(f"channel must be one of {FIN_CHANNELS}")
    side = str(end).strip().lower()
    if side not in ("min", "max"):
        raise ValueError("end must be min or max")
    t = clamp_tick(tick)
    try:
        with open(PINS_PATH) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        data = {}
    pca = data.setdefault("pca9685", {})
    travel = pca.setdefault("servo_travel_ticks", {})
    key = str(ch)
    entry = dict(travel.get(key, travel.get(ch, {})) or {})
    if not isinstance(entry, dict):
        entry = {}
    entry[side] = t
    center = read_servo_center_ticks().get(ch, DEFAULT_CENTER_TICK)
    t_min = clamp_tick(int(entry.get("min", SERVO_MIN_TICK)))
    t_max = clamp_tick(int(entry.get("max", SERVO_MAX_TICK)))
    t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
    entry["min"] = t_min
    entry["max"] = t_max
    travel[key] = entry
    with open(PINS_PATH, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    return {"channel": ch, "end": side, "tick": t, "min_tick": t_min, "max_tick": t_max}


def format_travel_tune_command(channel: int, end: str, tick: int) -> str:
    side = str(end).strip().upper()
    if side not in ("MIN", "MAX"):
        raise ValueError("end must be min or max")
    return f"CAL S TUNE {side} {int(channel)} {clamp_tick(tick)}"


def format_travel_save_command(channel: int, end: str, tick: int) -> str:
    side = str(end).strip().upper()
    if side not in ("MIN", "MAX"):
        raise ValueError("end must be min or max")
    return f"CAL S {side} {int(channel)} {clamp_tick(tick)}"


def sync_travel_commands() -> list[str]:
    cmds: list[str] = []
    for ch, (t_min, t_max) in sorted(read_servo_travel_ticks().items()):
        cmds.append(format_travel_save_command(ch, "min", t_min))
        cmds.append(format_travel_save_command(ch, "max", t_max))
    return cmds


def fin_right_inverted_solo_command(
    fin_right_command: float,
    *,
    right_center: int,
    right_min: int | None = None,
    right_max: int | None = None,
) -> float:
    """Single fin R command with invert_command (manual slider / one-fin jog)."""
    d = tick_delta_for_command(
        fin_right_command,
        right_center,
        min_tick=right_min,
        max_tick=right_max,
    )
    return command_for_tick_delta(
        -d,
        right_center,
        min_tick=right_min,
        max_tick=right_max,
    )


def wire_fin_commands_for_s2(
    fin_left: float,
    fin_right: float,
    *,
    invert_fin_r: bool,
) -> tuple[float, float]:
    """
    Logical fin commands → S2 wire values (all Xbox / manual / auto modes).

    invert_command uses matched tick travel on the common (pitch) component plus roll diff.
    """
    fl = max(-1.0, min(1.0, float(fin_left)))
    fr = max(-1.0, min(1.0, float(fin_right)))
    if not invert_fin_r:
        return fl, fr

    centers = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    c4 = centers[4]
    c5 = centers[5]
    l_min, l_max = travel[4]
    r_min, r_max = travel[5]

    if abs(fl) < 1e-6 and abs(fr) >= 1e-6:
        return fl, fin_right_inverted_solo_command(
            fr,
            right_center=c5,
            right_min=r_min,
            right_max=r_max,
        )

    common = (fl + fr) * 0.5
    diff = (fl - fr) * 0.5
    fr_wire = fin_right_inverted_matched_command(
        common,
        left_center=c4,
        right_center=c5,
        left_min=l_min,
        left_max=l_max,
        right_min=r_min,
        right_max=r_max,
    ) + diff
    return fl, max(-1.0, min(1.0, fr_wire))


def clamp_channel_s2_command(v: float, channel: int) -> float:
    """
    Normalize a logical -1..1 command through yaml centre/travel (same as ESP).
    Use on fin/aft channels so saved max/min limits apply on the wire.
    """
    v = max(-1.0, min(1.0, float(v)))
    if abs(v) < 1e-9:
        return 0.0
    tick = tick_for_channel_command(v, channel)
    center = read_servo_center_ticks()[int(channel)]
    t_min, t_max = read_servo_travel_ticks()[int(channel)]
    if tick >= center:
        span = float(t_max - center)
        if span <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, (tick - center) / span))
    span = float(center - t_min)
    if span <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, (tick - center) / span))


def tick_for_channel_command(v: float, channel: int) -> int:
    """Match esp32 sub_rc servoTickForChannel (yaml centre + travel limits)."""
    ch = int(channel)
    centers = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    center = centers[ch]
    t_min, t_max = travel[ch]
    t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
    v = max(-1.0, min(1.0, float(v)))
    if v >= 0.0:
        pulse = float(center) + v * float(t_max - center)
    else:
        pulse = float(center) + v * float(center - t_min)
    pulse = max(float(t_min), min(float(t_max), pulse))
    return int(pulse + 0.5)


def command_for_channel_tick(tick: int, channel: int) -> float:
    """Inverse of tick_for_channel_command for one side of travel from centre."""
    ch = int(channel)
    centers = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    center = centers[ch]
    t_min, t_max = travel[ch]
    t = clamp_tick(tick)
    t_min, t_max = _sanitize_travel_ticks(center, t_min, t_max)
    if t >= center:
        span = float(t_max - center)
        if span <= 0.0:
            return 0.0
        return max(-1.0, min(1.0, (t - center) / span))
    span = float(center - t_min)
    if span <= 0.0:
        return 0.0
    return max(-1.0, min(1.0, (t - center) / span))


def wire_s2_actuators(
    act: "SubActuators",
    *,
    invert_fin_r: bool,
) -> "SubActuators":
    """
    Map logical actuator commands to S2 wire values using per-channel travel limits
    (same mapping as ESP servoTickForChannel) for every control mode.
    """
    from src.sub_state import SubActuators

    fl, fr = wire_fin_commands_for_s2(
        act.fin_left,
        act.fin_right,
        invert_fin_r=invert_fin_r,
    )
    return SubActuators(
        aft_steer_y=clamp_channel_s2_command(act.aft_steer_y, 3),
        aft_steer_z=clamp_channel_s2_command(act.aft_steer_z, 2),
        thruster_x=max(-1.0, min(1.0, float(act.thruster_x))),
        fin_left=clamp_channel_s2_command(fl, 4),
        fin_right=clamp_channel_s2_command(fr, 5),
    )


def read_servo_center_ticks() -> dict[int, int]:
    try:
        with open(PINS_PATH) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        data = {}
    default = _default_tick_from_legacy(data)
    raw = (data.get("pca9685") or {}).get("servo_center_ticks") or {}
    out: dict[int, int] = {}
    for ch in FIN_CHANNELS:
        val = raw.get(ch, raw.get(str(ch), default))
        out[ch] = clamp_tick(int(val))
    return out


def write_servo_center_tick(channel: int, tick: int) -> dict[str, int]:
    ch = int(channel)
    if ch not in FIN_CHANNELS:
        raise ValueError(f"channel must be one of {FIN_CHANNELS}")
    t = clamp_tick(tick)
    try:
        with open(PINS_PATH) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        data = {}
    pca = data.setdefault("pca9685", {})
    centers = pca.setdefault("servo_center_ticks", {})
    centers[str(ch)] = t
    with open(PINS_PATH, "w") as f:
        yaml.dump(data, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    return {"channel": ch, "centre_tick": t}


def format_tune_command(channel: int, tick: int) -> str:
    return f"CAL S TUNE {int(channel)} {clamp_tick(tick)}"


def format_center_command(channel: int, tick: int) -> str:
    return f"CAL S {int(channel)} {clamp_tick(tick)}"


def format_tune_pwm_command(channel: int, tick: int) -> str:
    return f"TEST P {int(channel)} {clamp_tick(tick)}"


def sync_center_commands() -> list[str]:
    return [format_center_command(ch, t) for ch, t in sorted(read_servo_center_ticks().items())]


def sync_all_servo_cal_commands() -> list[str]:
    return sync_center_commands() + sync_travel_commands()


def centers_snapshot() -> dict:
    ticks = read_servo_center_ticks()
    travel = read_servo_travel_ticks()
    channels = []
    for ch in FIN_CHANNELS:
        t = ticks[ch]
        t_min, t_max = travel[ch]
        channels.append({
            "channel": ch,
            "label": CHANNEL_LABELS.get(ch, f"Ch {ch}"),
            "centre_tick": t,
            "centre_deg": round(tick_to_deg(t), 1),
            "travel_min_tick": t_min,
            "travel_max_tick": t_max,
            "travel_min_deg": round(tick_to_deg(t_min), 1),
            "travel_max_deg": round(tick_to_deg(t_max), 1),
            "tick_min": SERVO_MIN_TICK,
            "tick_max": SERVO_MAX_TICK,
        })
    return {"channels": channels}
