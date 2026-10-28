"""
Map MPU6050 chip-frame pitch/roll into the hull frame.

sub_rc.ino reports:

  pitch = atan2(ay, hypot(ax, az))
  roll  = atan2(-ax, az)

If the GY-521 sits on its side, gravity lands on chip X or Y and the
dashboard shows ~90° of tilt with the horizon rotated on its side.
Reconstruct the unit gravity vector, permute axes, then recompute
the same Euler pair.

``imu.mount: auto`` (default) picks the permutation that levels the
rest pose. An explicit 0–4 is used only when it does not make rest
tilt *worse* than the raw chip reading — so a leftover ``mount: 1``
cannot turn a flat board into a 90° roll. ``imu.yaw: 90`` swaps
pitch/roll after that; it is not a substitute for a leveling mount.
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Callable

import yaml

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"

# Chip accel (x, y, z) → vehicle accel. Vehicle: X forward, Y starboard, Z up.
_MOUNTS: dict[int, Callable[[float, float, float], tuple[float, float, float]]] = {
    0: lambda ax, ay, az: (ax, ay, az),
    1: lambda ax, ay, az: (az, ay, -ax),   # +90° about Y (chip -X up)
    2: lambda ax, ay, az: (-az, ay, ax),  # -90° about Y (chip +X up)
    3: lambda ax, ay, az: (ax, -az, ay),  # +90° about X (chip +Y up)
    4: lambda ax, ay, az: (ax, az, -ay),  # -90° about X (chip -Y up)
    5: lambda ax, ay, az: (-ay, ax, az),  # +90° yaw (swap pitch/roll)
    6: lambda ax, ay, az: (ay, -ax, az),  # -90° yaw
}

_LEVEL_MOUNTS = (0, 1, 2, 3, 4)
_LOCK_SAMPLES = 8
_WORSEN_DEG = 20.0
_RELOCK_WINDOW_S = 12.0
_RELOCK_HULL_TILT = 50.0

_cfg_cache: tuple[float, object] | None = None
_lock_mount: int | None = None
_lock_yaw: int = 0
_lock_at: float = 0.0
_samples: list[tuple[float, float]] = []
_logged = False


def gravity_from_pitch_roll(pitch_deg: float, roll_deg: float) -> tuple[float, float, float]:
    """Unit gravity vector implied by the firmware Euler formulas."""
    pitch = math.radians(float(pitch_deg))
    roll = math.radians(float(roll_deg))
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cr = math.cos(roll)
    sr = math.sin(roll)
    ax = -cp * sr
    ay = sp
    az = cp * cr
    return ax, ay, az


def pitch_roll_from_gravity(ax: float, ay: float, az: float) -> tuple[float, float]:
    """Same Euler pair as esp32/sub_rc/sub_rc.ino readMpu6050()."""
    pitch = math.degrees(math.atan2(ay, math.hypot(ax, az)))
    roll = math.degrees(math.atan2(-ax, az))
    return pitch, roll


def remap_attitude(
    pitch_deg: float,
    roll_deg: float,
    *,
    mount: int = 0,
    yaw_mount: int = 0,
) -> tuple[float, float]:
    ax, ay, az = gravity_from_pitch_roll(pitch_deg, roll_deg)
    mapper = _MOUNTS.get(int(mount), _MOUNTS[0])
    vx, vy, vz = mapper(ax, ay, az)
    yaw_id = int(yaw_mount or 0)
    if yaw_id in (5, 6):
        vx, vy, vz = _MOUNTS[yaw_id](vx, vy, vz)
    return pitch_roll_from_gravity(vx, vy, vz)


def tilt_deg(pitch_deg: float, roll_deg: float) -> float:
    return math.hypot(float(pitch_deg), float(roll_deg))


def choose_level_mount(pitch_deg: float, roll_deg: float) -> int:
    """Gravity permutation (0–4) that puts +Z up. Yaw swaps are not leveling."""
    best_m = 0
    best = tilt_deg(pitch_deg, roll_deg)
    for mount in _LEVEL_MOUNTS:
        hull_p, hull_r = remap_attitude(pitch_deg, roll_deg, mount=mount)
        t = tilt_deg(hull_p, hull_r)
        if t + 1e-6 < best:
            best = t
            best_m = mount
    return best_m


def reset_imu_mapper() -> None:
    """Tests / config reload: forget the locked rest-pose mount."""
    global _cfg_cache, _lock_mount, _lock_yaw, _lock_at, _samples, _logged
    _cfg_cache = None
    _lock_mount = None
    _lock_yaw = 0
    _lock_at = 0.0
    _samples = []
    _logged = False


def _load_imu_cfg(path: Path = CONFIG_PATH) -> dict:
    global _cfg_cache
    now = __import__("time").monotonic()
    if _cfg_cache is not None and now - _cfg_cache[0] < 2.0:
        return dict(_cfg_cache[1])
    cfg: dict = {"mount": "auto", "yaw": 0}
    try:
        with open(path) as handle:
            raw = yaml.safe_load(handle) or {}
        imu = raw.get("imu") or {}
        cfg["mount"] = imu.get("mount", "auto")
        yaw = imu.get("yaw", 0)
        if yaw in (90, "90", "+90"):
            cfg["yaw"] = 5
        elif yaw in (-90, "-90"):
            cfg["yaw"] = 6
        else:
            cfg["yaw"] = int(yaw or 0)
            if cfg["yaw"] not in (0, 5, 6):
                cfg["yaw"] = 0
    except (OSError, TypeError, ValueError):
        pass
    _cfg_cache = (now, cfg)
    return dict(cfg)


def imu_mount_from_config(path: Path = CONFIG_PATH) -> int:
    """Locked mount id, or 0 before the rest pose is known."""
    if _lock_mount is not None:
        return _lock_mount
    cfg = _load_imu_cfg(path)
    mount = cfg.get("mount", "auto")
    if mount in ("auto", None, ""):
        return 0
    try:
        return int(mount)
    except (TypeError, ValueError):
        return 0


def _resolve_mount(avg_p: float, avg_r: float, configured) -> int:
    raw_tilt = tilt_deg(avg_p, avg_r)
    if configured not in ("auto", None, ""):
        try:
            wanted = int(configured)
        except (TypeError, ValueError):
            wanted = -1
        if wanted in _MOUNTS:
            hull_p, hull_r = remap_attitude(avg_p, avg_r, mount=wanted)
            if tilt_deg(hull_p, hull_r) <= raw_tilt + _WORSEN_DEG:
                return wanted
            print(
                f"[imu] configured mount={wanted} leaves rest tilt "
                f"{tilt_deg(hull_p, hull_r):.0f}° (chip {raw_tilt:.0f}°); "
                "picking a leveling mount instead"
            )
    return choose_level_mount(avg_p, avg_r)


def _should_relock(
    chip_p: float,
    chip_r: float,
    hull_p: float,
    hull_r: float,
) -> bool:
    """True when we locked 'flat' on zeros, then the chip came up on its side."""
    if _lock_mount is None or _lock_at <= 0:
        return False
    if time.monotonic() - _lock_at > _RELOCK_WINDOW_S:
        return False
    hull_t = tilt_deg(hull_p, hull_r)
    if hull_t < _RELOCK_HULL_TILT:
        return False
    better = choose_level_mount(chip_p, chip_r)
    if better == _lock_mount:
        return False
    improved = remap_attitude(
        chip_p, chip_r, mount=better, yaw_mount=_lock_yaw
    )
    return tilt_deg(*improved) + 20.0 < hull_t


def map_reported_attitude(
    pitch_deg: float,
    roll_deg: float,
    *,
    path: Path = CONFIG_PATH,
) -> tuple[float, float]:
    """Chip Euler → hull Euler, locking the rest-pose mount after a few samples."""
    global _lock_mount, _lock_yaw, _lock_at, _logged
    cfg = _load_imu_cfg(path)
    yaw_mount = int(cfg.get("yaw") or 0)

    if _lock_mount is not None:
        hull = remap_attitude(
            pitch_deg, roll_deg, mount=_lock_mount, yaw_mount=_lock_yaw
        )
        if not _should_relock(pitch_deg, roll_deg, *hull):
            return hull
        print(
            f"[imu] relocking (hull still {tilt_deg(*hull):.0f}°); "
            "chip was probably not ready when the rest pose locked"
        )
        _lock_mount = None
        _lock_at = 0.0
        _samples.clear()
        _logged = False

    if _lock_mount is None:
        _samples.append((float(pitch_deg), float(roll_deg)))
        if len(_samples) < _LOCK_SAMPLES:
            preview = choose_level_mount(pitch_deg, roll_deg)
            return remap_attitude(
                pitch_deg, roll_deg, mount=preview, yaw_mount=yaw_mount
            )
        avg_p = sum(p for p, _ in _samples) / len(_samples)
        avg_r = sum(r for _, r in _samples) / len(_samples)
        _lock_mount = _resolve_mount(avg_p, avg_r, cfg.get("mount"))
        _lock_yaw = yaw_mount
        _lock_at = time.monotonic()
        hull_p, hull_r = remap_attitude(
            avg_p, avg_r, mount=_lock_mount, yaw_mount=_lock_yaw
        )
        if not _logged:
            _logged = True
            print(
                f"[imu] mount={_lock_mount} yaw={_lock_yaw} "
                f"rest chip P={avg_p:.1f} R={avg_r:.1f} "
                f"→ hull P={hull_p:.1f} R={hull_r:.1f}"
            )

    return remap_attitude(
        pitch_deg, roll_deg, mount=_lock_mount, yaw_mount=_lock_yaw
    )
