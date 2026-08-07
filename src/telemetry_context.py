"""
telemetry_context.py
--------------------
Read-only snapshot of ESP telemetry for the YOLO control loop.

Built from sub_state (updated by esp_bridge). Passed to Controller.compute()
for safety limits only (leak, battery). Steering, tilt, and forward drive
come from the camera — not from depth or other telemetry.
"""

from __future__ import annotations

import time
from dataclasses import dataclass


@dataclass(frozen=True)
class TelemetryContext:
    esp_connected: bool = False
    telemetry_age_s: float | None = None

    depth_m: float | None = None
    battery_v: float | None = None
    leak_triggered: bool = False

    thruster_value: float | None = None
    thruster_pwm: int | None = None

    pitch: float | None = None
    roll: float | None = None
    yaw: float | None = None

    ballast_fore_level: float | None = None
    ballast_aft_level: float | None = None

    @property
    def fresh(self) -> bool:
        """True when ESP telemetry arrived recently enough to trust."""
        if not self.esp_connected:
            return False
        if self.telemetry_age_s is None:
            return False
        return self.telemetry_age_s < 3.0

    @property
    def depth_valid(self) -> bool:
        return self.fresh and self.depth_m is not None

    @classmethod
    def from_sub_state(cls) -> TelemetryContext:
        from src.sub_state import get_sub_state

        snap = get_sub_state().telemetry_snapshot()
        gyro = snap.get("gyro") or {}
        ballast = snap.get("ballast") or {}
        fore = ballast.get("fore") or {}
        aft = ballast.get("aft") or {}
        thr = snap.get("thruster") or {}
        bat = snap.get("battery") or {}
        depth = snap.get("depth") or {}
        leaks = snap.get("leaks") or {}

        return cls(
            esp_connected=bool(snap.get("esp_connected")),
            telemetry_age_s=snap.get("telemetry_age_s"),
            depth_m=depth.get("meters"),
            battery_v=bat.get("voltage"),
            leak_triggered=bool(leaks.get("triggered")),
            thruster_value=thr.get("value"),
            thruster_pwm=thr.get("pwm"),
            pitch=gyro.get("pitch"),
            roll=gyro.get("roll"),
            yaw=gyro.get("yaw"),
            ballast_fore_level=fore.get("level"),
            ballast_aft_level=aft.get("level"),
        )

    @classmethod
    def empty(cls) -> TelemetryContext:
        return cls()
