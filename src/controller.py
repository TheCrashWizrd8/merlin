"""
controller.py
-------------
Converts a TrackResult into a ControlOutput — the single source of
truth for all actuator commands.

All output values are in the normalised range -1.0 … +1.0.
The hardware layer (when wired up) maps these to actual PWM microseconds
using the ranges defined in config/hardware.yaml.

Control logic (proportional only for now)
-----------------------------------------
  steering_servo    = gain_steer  * error_x
  camera_tilt_servo = gain_tilt   * error_y   (inverted: apple below → tilt down)
  drive_motor       = gain_drive  * (1 - |error_x|)
                      → slow down when turning sharply; stop when no apple

Gains are deliberately conservative defaults.  Tune them once hardware
is connected by adjusting the constants below or by passing them in.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional

import yaml

from src.tracker import TrackResult


CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"

# Default proportional gains (override via constructor)
DEFAULT_GAIN_STEER: float = 1.0   # full error → full steering
DEFAULT_GAIN_TILT: float = 0.8    # slightly gentler camera movement
DEFAULT_GAIN_DRIVE: float = 0.6   # max forward speed at 60% while developing

CONTROL_PROFILES: dict[str, dict[str, float | int]] = {
    # More damping/smoothing, stricter confidence.
    "stable": {
        "smoothing_alpha": 0.22,
        "damping_steer": 0.30,
        "damping_tilt": 0.24,
        "min_detection_confidence": 0.50,
        "hold_missed_frames": 7,
        "max_drive_when_turning": 0.48,
        "min_drive_command": 0.50,
    },
    # Faster response, less filtering.
    "aggressive": {
        "smoothing_alpha": 0.45,
        "damping_steer": 0.12,
        "damping_tilt": 0.10,
        "min_detection_confidence": 0.35,
        "hold_missed_frames": 3,
        "max_drive_when_turning": 0.60,
        "min_drive_command": 0.55,
    },
}


@dataclass
class ControlOutput:
    """
    Complete actuator command set for one inference frame.

    All servo/motor fields are normalised: -1.0 … +1.0
      steering_servo:    -1.0 = full left,    +1.0 = full right
      drive_motor:       -1.0 = full reverse, +1.0 = full forward
      camera_tilt_servo: -1.0 = full down,    +1.0 = full up
    """
    # Actuator commands
    steering_servo: float
    drive_motor: float
    camera_tilt_servo: float
    # Detection info
    apple_detected: bool
    target_x: int
    target_y: int
    error_x: float
    error_y: float
    confidence: float
    # Bounding box of tracked target (for display); (0,0,0,0) when none
    bbox_x1: int = 0
    bbox_y1: int = 0
    bbox_x2: int = 0
    bbox_y2: int = 0
    # Bbox size for distance estimation (larger = closer)
    bbox_width: int = 0
    bbox_height: int = 0
    bbox_area: int = 0
    frame_area: int = 0
    # bbox_area/frame_area: raw (instant) vs filtered (used for size-based drive)
    size_ratio_raw: float = 0.0
    size_ratio_filtered: float = 0.0
    chosen_label: str = ""
    # Timing
    timestamp: float = 0.0

    def to_dict(self) -> dict:
        """Return a plain dict (useful for serialisation / passing to hardware)."""
        return asdict(self)

    def pretty(self) -> str:
        """
        Return a compact, human-readable table string suitable for
        printing to the terminal every frame.

        Example output
        --------------
        ┌─────────────────────────────────────────────┐
        │  CONTROL OUTPUT          2024-01-01 12:00:00 │
        ├──────────────────┬──────────────────────────┤
        │ apple_detected   │  YES                      │
        │ target           │  x=320  y=240             │
        │ error_x          │  +0.042  →ₓ               │
        │ error_y          │  -0.115  ↑ᵧ               │
        │ confidence       │  0.87                     │
        ├──────────────────┼──────────────────────────┤
        │ steering_servo   │  +0.042  (right)          │
        │ drive_motor      │  +0.560  (forward)        │
        │ camera_tilt      │  +0.092  (up)             │
        └──────────────────┴──────────────────────────┘
        """
        from datetime import datetime
        ts = datetime.fromtimestamp(self.timestamp).strftime("%H:%M:%S.%f")[:-3]
        detected_str = "YES" if self.apple_detected else "NO"

        def bar(value: float, width: int = 20) -> str:
            """Mini ASCII bar: centre = 0, left = negative, right = positive."""
            half = width // 2
            pos = int(round(value * half))
            pos = max(-half, min(half, pos))
            left = half + pos
            bar_chars = ["-"] * width
            bar_chars[half] = "|"
            if pos != 0:
                for i in range(min(half, left), max(half, left)):
                    bar_chars[i] = "█"
            return "".join(bar_chars)

        def fmt_servo(label: str, value: float, lo: str, hi: str) -> str:
            direction = hi if value > 0 else (lo if value < 0 else "centre")
            return f"│ {label:<17} │ {value:+.3f}  {bar(value)}  ({direction})"

        w = 60
        sep_top    = "┌" + "─" * (w - 2) + "┐"
        sep_mid    = "├" + "─" * 19 + "┬" + "─" * (w - 22) + "┤"
        sep_mid2   = "├" + "─" * 19 + "┼" + "─" * (w - 22) + "┤"
        sep_bot    = "└" + "─" * 19 + "┴" + "─" * (w - 22) + "┘"

        header = f"│  CONTROL OUTPUT  {ts:>{w - 22}}"
        header = header + " " * (w - 1 - len(header)) + "│"

        lines = [
            sep_top,
            header,
            sep_mid,
            f"│ {'apple_detected':<17} │ {detected_str:<{w - 23}}│",
            f"│ {'target':<17} │ x={self.target_x:<5} y={self.target_y:<{w - 35}}│",
            f"│ {'error_x':<17} │ {self.error_x:+.4f}{'':<{w - 28}}│",
            f"│ {'error_y':<17} │ {self.error_y:+.4f}{'':<{w - 28}}│",
            f"│ {'confidence':<17} │ {self.confidence:.4f}{'':<{w - 28}}│",
            sep_mid2,
            fmt_servo("steering_servo", self.steering_servo,  "left",    "right")   + " " * max(0, w - 54) + "│",
            fmt_servo("drive_motor",    self.drive_motor,     "reverse", "forward") + " " * max(0, w - 57) + "│",
            fmt_servo("camera_tilt",    self.camera_tilt_servo, "down",  "up")      + " " * max(0, w - 52) + "│",
            sep_bot,
        ]
        return "\n".join(lines)


class Controller:
    """
    Converts a TrackResult into a ControlOutput.

    Parameters
    ----------
    gain_steer : float
        Proportional gain for steering.  1.0 = full error → full lock.
    gain_tilt : float
        Proportional gain for camera tilt.
    gain_drive : float
        Base drive speed multiplier when apple is centred.
    deadzone : float
        Errors below this magnitude are treated as zero.
    """

    def __init__(
        self,
        gain_steer: float = DEFAULT_GAIN_STEER,
        gain_tilt: float = DEFAULT_GAIN_TILT,
        gain_drive: float = DEFAULT_GAIN_DRIVE,
        deadzone: float = 0.05,
        min_steer_command: float = 0.0,
        min_tilt_command: float = 0.0,
        min_drive_command: float = 0.0,
        min_detection_confidence: float = 0.35,
        hold_missed_frames: int = 4,
        smoothing_alpha: float = 0.35,
        damping_steer: float = 0.25,
        damping_tilt: float = 0.20,
        max_drive_when_turning: float = 0.30,
        use_size_for_drive: bool = False,
        size_min_ratio: float = 0.005,
        size_max_ratio: float = 0.12,
        size_smoothing_alpha: float = 0.22,
        size_curve: str = "sqrt",
        size_drive_far: float = 1.0,
        size_drive_close: float = 0.0,
    ) -> None:
        self.gain_steer = gain_steer
        self.gain_tilt = gain_tilt
        self.gain_drive = gain_drive
        self.deadzone = deadzone
        self.min_steer_command = max(0.0, min(1.0, min_steer_command))
        self.min_tilt_command = max(0.0, min(1.0, min_tilt_command))
        self.min_drive_command = max(0.0, min(1.0, min_drive_command))
        self.min_detection_confidence = max(0.0, min(1.0, min_detection_confidence))
        self.hold_missed_frames = max(0, int(hold_missed_frames))
        self.smoothing_alpha = max(0.0, min(1.0, smoothing_alpha))
        self.damping_steer = max(0.0, damping_steer)
        self.damping_tilt = max(0.0, damping_tilt)
        self.max_drive_when_turning = max(0.0, min(1.0, max_drive_when_turning))
        self.use_size_for_drive = bool(use_size_for_drive)
        self.size_min_ratio = max(1e-6, float(size_min_ratio))
        self.size_max_ratio = max(self.size_min_ratio, float(size_max_ratio))
        self.size_smoothing_alpha = max(0.0, min(1.0, float(size_smoothing_alpha)))
        sc = (size_curve or "sqrt").strip().lower()
        if sc not in ("linear", "sqrt"):
            raise ValueError("size_curve must be 'linear' or 'sqrt'")
        self.size_curve = sc
        self.size_drive_far = float(size_drive_far)
        self.size_drive_close = float(size_drive_close)

        # Internal state for smoothing, derivative damping and persistence.
        self._filtered_ex = 0.0
        self._filtered_ey = 0.0
        self._prev_filtered_ex = 0.0
        self._prev_filtered_ey = 0.0
        self._last_t_monotonic: float | None = None
        self._last_valid_track: Optional[TrackResult] = None
        self._missed_frames = 0
        self._filtered_size_ratio: float = 0.0

    @classmethod
    def from_hardware_config(
        cls,
        config_path: Path = CONFIG_PATH,
        profile: str = "config",
    ) -> "Controller":
        """Load controller gains/deadzone from hardware.yaml, with optional profile overrides."""
        with open(config_path, "r") as f:
            cfg = yaml.safe_load(f)
        params: dict[str, float | int] = {
            "gain_steer": float(cfg.get("gain_steer", DEFAULT_GAIN_STEER)),
            "gain_tilt": float(cfg.get("gain_tilt", DEFAULT_GAIN_TILT)),
            "gain_drive": float(cfg.get("max_drive_speed", DEFAULT_GAIN_DRIVE)),
            "deadzone": float(cfg.get("deadzone", 0.05)),
            "min_steer_command": float(cfg.get("min_steer_command", 0.0)),
            "min_tilt_command": float(cfg.get("min_tilt_command", 0.0)),
            "min_drive_command": float(cfg.get("min_drive_command", 0.0)),
            "min_detection_confidence": float(cfg.get("min_detection_confidence", 0.35)),
            "hold_missed_frames": int(cfg.get("hold_missed_frames", 4)),
            "smoothing_alpha": float(cfg.get("smoothing_alpha", 0.35)),
            "damping_steer": float(cfg.get("damping_steer", 0.25)),
            "damping_tilt": float(cfg.get("damping_tilt", 0.20)),
            "max_drive_when_turning": float(cfg.get("max_drive_when_turning", 0.30)),
            "use_size_for_drive": bool(cfg.get("use_size_for_drive", False)),
            "size_min_ratio": float(cfg.get("size_min_ratio", 0.005)),
            "size_max_ratio": float(cfg.get("size_max_ratio", 0.12)),
            "size_smoothing_alpha": float(cfg.get("size_smoothing_alpha", 0.22)),
            "size_curve": str(cfg.get("size_curve", "sqrt")),
            "size_drive_far": float(cfg.get("size_drive_far", 1.0)),
            "size_drive_close": float(cfg.get("size_drive_close", 0.0)),
        }
        profile = (profile or "config").strip().lower()
        if profile != "config":
            if profile not in CONTROL_PROFILES:
                raise ValueError(
                    f"Unknown control profile '{profile}'. Use one of: config, stable, aggressive."
                )
            params.update(CONTROL_PROFILES[profile])
        return cls(**params)

    def _apply_deadzone(self, value: float) -> float:
        return 0.0 if abs(value) < self.deadzone else value

    def _clamp(self, value: float) -> float:
        return max(-1.0, min(1.0, value))

    def _size_ratio_to_drive_t(self, area_ratio: float) -> float:
        """
        Map bbox_area/frame_area to [0, 1] for interpolating drive.

        t≈0: small bbox (far). t≈1: large bbox (close).
        Caller maps t to drive with closer → lower D.

        'sqrt': use sqrt(area_ratio) — closer to linear pixel size / distance
                than raw area (area ~ 1/d² for a fixed physical object).
        'linear': raw area ratio (legacy).
        """
        lo = self.size_min_ratio
        hi = self.size_max_ratio
        a = max(0.0, area_ratio)
        if self.size_curve == "sqrt":
            ra = math.sqrt(a)
            rlo = math.sqrt(lo)
            rhi = math.sqrt(hi)
            if rhi <= rlo:
                return 0.0
            t = (ra - rlo) / (rhi - rlo)
        else:
            span = hi - lo
            if span <= 0:
                return 0.0
            t = (a - lo) / span
        return max(0.0, min(1.0, t))

    def _with_min_command(self, value: float, minimum: float) -> float:
        """Keep motion alive when non-zero error exists, instead of tiny bursts."""
        if value == 0.0:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return sign * max(abs(value), minimum)

    def compute(self, track: TrackResult) -> ControlOutput:
        """
        Compute actuator commands from a TrackResult.

        When no apple is detected all outputs are zero (stop / hold).
        """
        now_monotonic = time.monotonic()
        dt = 1.0 / 30.0
        if self._last_t_monotonic is not None:
            dt = max(1e-3, now_monotonic - self._last_t_monotonic)
        self._last_t_monotonic = now_monotonic

        has_confident_detection = (
            track.apple_detected and track.confidence >= self.min_detection_confidence
        )

        regained_track = False
        if has_confident_detection:
            regained_track = self._last_valid_track is None
            self._last_valid_track = track
            self._missed_frames = 0
            active = track
        else:
            self._missed_frames += 1
            # Persistence: keep steering/tilt/motion briefly on dropouts.
            if self._last_valid_track is not None and self._missed_frames <= self.hold_missed_frames:
                active = self._last_valid_track
            else:
                self._filtered_ex = 0.0
                self._filtered_ey = 0.0
                self._prev_filtered_ex = 0.0
                self._prev_filtered_ey = 0.0
                self._last_valid_track = None
                self._filtered_size_ratio = 0.0
                return ControlOutput(
                    steering_servo=0.0,
                    drive_motor=0.0,
                    camera_tilt_servo=0.0,
                    apple_detected=False,
                    target_x=0,
                    target_y=0,
                    error_x=0.0,
                    error_y=0.0,
                    confidence=0.0,
                    bbox_x1=0,
                    bbox_y1=0,
                    bbox_x2=0,
                    bbox_y2=0,
                    chosen_label="",
                    timestamp=time.time(),
                )

        raw_ex = active.error_x
        raw_ey = active.error_y

        # Smoothing layer: EMA on error signals before control.
        a = self.smoothing_alpha
        self._filtered_ex = a * raw_ex + (1.0 - a) * self._filtered_ex
        self._filtered_ey = a * raw_ey + (1.0 - a) * self._filtered_ey

        ex = self._apply_deadzone(self._filtered_ex)
        ey = self._apply_deadzone(self._filtered_ey)

        # Sustain-until-centred: apply constant S/D/T while error exists; only stop when in deadzone.
        # Servos and motor keep moving at a fixed rate until centred — no proportional ramp-down.
        if abs(ex) < self.deadzone:
            steering = 0.0
        else:
            steering = (1.0 if ex > 0 else -1.0) * self.min_steer_command

        if abs(ey) < self.deadzone:
            tilt = 0.0
        else:
            tilt = (-1.0 if ey > 0 else 1.0) * self.min_tilt_command

        # Size ratio: update EMA only on fresh detections (not hold frames)
        size_raw = 0.0
        size_filt = self._filtered_size_ratio
        if active.frame_area > 0 and active.bbox_area > 0:
            size_raw = active.bbox_area / active.frame_area
            if has_confident_detection:
                sa = self.size_smoothing_alpha
                if regained_track:
                    self._filtered_size_ratio = size_raw
                else:
                    self._filtered_size_ratio = (
                        sa * size_raw + (1.0 - sa) * self._filtered_size_ratio
                    )
                size_filt = self._filtered_size_ratio

        # Drive: size-based — farther (small bbox) → more drive; closer (large) → less
        # Uses full [-1,1] span via size_drive_far / size_drive_close (not min/max_drive_command).
        if self.use_size_for_drive and active.frame_area > 0 and active.bbox_area > 0:
            t = self._size_ratio_to_drive_t(size_filt)
            far = max(-1.0, min(1.0, self.size_drive_far))
            close = max(-1.0, min(1.0, self.size_drive_close))
            # t=0 far → far; t=1 close → close
            drive = far - t * (far - close)
            drive = self._clamp(drive)
        else:
            drive = self._clamp(self.min_drive_command)

        return ControlOutput(
            steering_servo=steering,
            drive_motor=drive,
            camera_tilt_servo=tilt,
            apple_detected=True,
            target_x=active.target_x,
            target_y=active.target_y,
            error_x=self._filtered_ex,
            error_y=self._filtered_ey,
            confidence=active.confidence if has_confident_detection else 0.0,
            bbox_x1=active.bbox_x1,
            bbox_y1=active.bbox_y1,
            bbox_x2=active.bbox_x2,
            bbox_y2=active.bbox_y2,
            bbox_width=active.bbox_width,
            bbox_height=active.bbox_height,
            bbox_area=active.bbox_area,
            frame_area=active.frame_area,
            size_ratio_raw=size_raw,
            size_ratio_filtered=size_filt,
            chosen_label=active.chosen_label,
            timestamp=time.time(),
        )
