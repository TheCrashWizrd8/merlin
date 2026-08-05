"""
sub_state.py
------------
Thread-safe shared state for the sub dashboard: ESP telemetry, control axes,
serial log, and connection flags.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from typing import Any


@dataclass
class GyroReading:
    pitch: float | None = None
    roll: float | None = None
    yaw: float | None = None


@dataclass
class SubActuators:
    aft_steer_y: float = 0.0
    aft_steer_z: float = 0.0
    thruster_x: float = 0.0
    fin_left: float = 0.0
    fin_right: float = 0.0

    def as_dict(self) -> dict[str, float]:
        return {
            "aftSteerY": self.aft_steer_y,
            "aftSteerZ": self.aft_steer_z,
            "thrusterX": self.thruster_x,
            "finLeft": self.fin_left,
            "finRight": self.fin_right,
        }


@dataclass
class XboxState:
    connected: bool = False
    name: str = ""
    left_stick_x: float = 0.0
    left_stick_y: float = 0.0
    right_stick_x: float = 0.0
    right_stick_y: float = 0.0
    triggers: dict[str, float] = field(default_factory=lambda: {"lt": 0.0, "rt": 0.0})
    buttons: dict[str, bool] = field(default_factory=dict)
    last_update: float = 0.0

    def as_dict(self) -> dict[str, Any]:
        return {
            "connected": self.connected,
            "name": self.name,
            "sticks": {
                "left": {"x": self.left_stick_x, "y": self.left_stick_y},
                "right": {"x": self.right_stick_x, "y": self.right_stick_y},
            },
            "triggers": dict(self.triggers),
            "buttons": dict(self.buttons),
            "last_update": self.last_update,
        }


class SubState:
    """Central store for sub telemetry and control."""

    def __init__(self, serial_log_size: int = 500) -> None:
        self._lock = threading.Lock()
        self._serial_log: deque[dict[str, Any]] = deque(maxlen=serial_log_size)

        # Connection flags
        self.esp_connected: bool = False
        self.esp_port: str = ""
        self.xbox = XboxState()

        # Telemetry (from ESP)
        self.battery_voltage: float | None = None
        self.gyro = GyroReading()
        self.depth_m: float | None = None
        self.leak_sensors: list[bool] = []
        self.leak_triggered: bool = False
        self.ballast_level: float | None = None  # 0..1 reported by ESP
        self.telemetry_timestamp: float = 0.0

        # Control
        self.control_mode: str = "xbox"  # xbox | manual | auto
        self.ballast_command: float = 0.0  # -1 drain .. +1 fill
        self.manual_actuators = SubActuators()
        self.auto_actuators = SubActuators()
        self.xbox_actuators = SubActuators()
        self.effective_actuators = SubActuators()

        # ESP diagnostic responses (PING, PINS, TEST)
        self.last_pong_ts: float | None = None
        self.esp_pins_lines: list[str] = []
        self.esp_pins_map: dict[str, str] = {}
        self._capturing_pins: bool = False
        self.last_diagnostic: str = ""
        self.last_diagnostic_ts: float = 0.0

    # ------------------------------------------------------------------
    # Serial log
    # ------------------------------------------------------------------

    def append_serial(self, direction: str, line: str) -> None:
        with self._lock:
            self._serial_log.append({
                "ts": time.time(),
                "dir": direction,
                "line": line,
            })

    def get_serial_log(self, limit: int = 100) -> list[dict[str, Any]]:
        with self._lock:
            items = list(self._serial_log)
        return items[-limit:]

    # ------------------------------------------------------------------
    # Telemetry updates (from ESP bridge)
    # ------------------------------------------------------------------

    def update_battery(self, voltage: float) -> None:
        with self._lock:
            self.battery_voltage = voltage
            self.telemetry_timestamp = time.time()
            self.esp_connected = True

    def update_gyro(self, pitch: float, roll: float, yaw: float) -> None:
        with self._lock:
            self.gyro = GyroReading(pitch=pitch, roll=roll, yaw=yaw)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True

    def update_depth(self, depth_m: float) -> None:
        with self._lock:
            self.depth_m = depth_m
            self.telemetry_timestamp = time.time()
            self.esp_connected = True

    def update_leaks(self, sensors: list[bool]) -> None:
        with self._lock:
            self.leak_sensors = list(sensors)
            self.leak_triggered = any(sensors)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True

    def update_ballast_level(self, level: float) -> None:
        with self._lock:
            self.ballast_level = max(0.0, min(1.0, level))
            self.telemetry_timestamp = time.time()
            self.esp_connected = True

    def set_esp_connected(self, connected: bool, port: str = "") -> None:
        with self._lock:
            self.esp_connected = connected
            if port:
                self.esp_port = port

    def begin_pins_capture(self) -> None:
        with self._lock:
            self._capturing_pins = True
            self.esp_pins_lines = []
            self.esp_pins_map = {}

    def append_pins_line(self, line: str) -> None:
        with self._lock:
            if not self._capturing_pins:
                return
            self.esp_pins_lines.append(line)
            if line.startswith("PIN "):
                rest = line[4:]
                if "=" in rest:
                    key, val = rest.split("=", 1)
                    self.esp_pins_map[key.strip()] = val.strip()

    def end_pins_capture(self) -> None:
        with self._lock:
            self._capturing_pins = False
            self.last_diagnostic = "OK PINS"
            self.last_diagnostic_ts = time.time()

    def is_capturing_pins(self) -> bool:
        with self._lock:
            return self._capturing_pins

    def set_last_pong(self) -> None:
        with self._lock:
            self.last_pong_ts = time.time()
            self.last_diagnostic = "OK PONG"
            self.last_diagnostic_ts = time.time()

    def set_last_diagnostic(self, line: str) -> None:
        with self._lock:
            self.last_diagnostic = line
            self.last_diagnostic_ts = time.time()

    def diagnostics_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "last_pong_ts": self.last_pong_ts,
                "last_diagnostic": self.last_diagnostic,
                "last_diagnostic_ts": self.last_diagnostic_ts,
                "esp_pins_lines": list(self.esp_pins_lines),
                "esp_pins_map": dict(self.esp_pins_map),
                "capturing_pins": self._capturing_pins,
            }

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def set_control_mode(self, mode: str) -> None:
        with self._lock:
            self.control_mode = mode if mode in ("xbox", "manual", "auto") else "xbox"

    def get_control_mode(self) -> str:
        with self._lock:
            return self.control_mode

    def set_ballast_command(self, value: float) -> None:
        with self._lock:
            self.ballast_command = max(-1.0, min(1.0, float(value)))

    def get_ballast_command(self) -> float:
        with self._lock:
            return self.ballast_command

    def set_manual_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            self.manual_actuators = actuators

    def set_auto_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            self.auto_actuators = actuators

    def set_xbox_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            self.xbox_actuators = actuators

    def recompute_effective(self) -> SubActuators:
        with self._lock:
            if self.control_mode == "auto":
                self.effective_actuators = SubActuators(**asdict(self.auto_actuators))
            elif self.control_mode == "manual":
                self.effective_actuators = SubActuators(**asdict(self.manual_actuators))
            else:
                self.effective_actuators = SubActuators(**asdict(self.xbox_actuators))
            return SubActuators(**asdict(self.effective_actuators))

    def update_xbox(self, xbox: XboxState) -> None:
        with self._lock:
            self.xbox = xbox

    # ------------------------------------------------------------------
    # API snapshots
    # ------------------------------------------------------------------

    def telemetry_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "esp_connected": self.esp_connected,
                "esp_port": self.esp_port,
                "timestamp": self.telemetry_timestamp,
                "battery": {
                    "voltage": self.battery_voltage,
                    "connected": self.battery_voltage is not None,
                },
                "gyro": {
                    "pitch": self.gyro.pitch,
                    "roll": self.gyro.roll,
                    "yaw": self.gyro.yaw,
                    "connected": self.gyro.pitch is not None,
                },
                "depth": {
                    "meters": self.depth_m,
                    "connected": self.depth_m is not None,
                },
                "leaks": {
                    "sensors": list(self.leak_sensors),
                    "triggered": self.leak_triggered,
                    "connected": len(self.leak_sensors) > 0,
                },
                "ballast": {
                    "level": self.ballast_level,
                    "command": self.ballast_command,
                    "connected": self.ballast_level is not None,
                },
            }

    def control_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "mode": self.control_mode,
                "ballast_command": self.ballast_command,
                "xbox": self.xbox.as_dict(),
                "effective": self.effective_actuators.as_dict(),
                "auto": self.auto_actuators.as_dict(),
                "manual": self.manual_actuators.as_dict(),
                "xbox_mapped": self.xbox_actuators.as_dict(),
                "timestamp": time.time(),
            }

    def status_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "esp_connected": self.esp_connected,
                "xbox_connected": self.xbox.connected,
                "leak_alarm": self.leak_triggered,
                "control_mode": self.control_mode,
                "telemetry_age_s": (
                    time.time() - self.telemetry_timestamp
                    if self.telemetry_timestamp > 0
                    else None
                ),
                "timestamp": time.time(),
            }


# Module singleton — shared across bridge, xbox, web
_state: SubState | None = None
_state_lock = threading.Lock()


def get_sub_state() -> SubState:
    global _state
    with _state_lock:
        if _state is None:
            _state = SubState()
        return _state
