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
from pathlib import Path
from typing import Any

import yaml

_PINS_YAML = Path(__file__).parent.parent / "config" / "pins.yaml"


def _load_leak_meta() -> tuple[list[int], list[str], bool]:
    try:
        with open(_PINS_YAML) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    leaks = (cfg.get("sub") or {}).get("leaks") or {}
    signal = leaks.get("signal_gpio")
    gpios = [int(signal)] if signal is not None else [int(g) for g in (leaks.get("gpios") or [5])]
    labels = [str(l) for l in (leaks.get("zones") or leaks.get("labels") or ["fore", "aft", "electronics", "battery"])]
    combined = bool(leaks.get("board") or len(gpios) <= 1)
    return gpios, labels, combined


@dataclass
class BallastTankState:
    level: float | None = None
    adc: int | None = None
    moving: bool = False
    direction: str = "STOP"
    command: float = 0.0
    cal_top_adc: int | None = None
    cal_bottom_adc: int | None = None
    cal_valid: bool = False

    def as_dict(self) -> dict[str, Any]:
        return {
            "level": self.level,
            "adc": self.adc,
            "moving": self.moving,
            "direction": self.direction,
            "command": self.command,
            "connected": self.adc is not None and self.adc >= 0,
            "cal_top_adc": self.cal_top_adc,
            "cal_bottom_adc": self.cal_bottom_adc,
            "cal_valid": self.cal_valid,
        }


@dataclass
class GyroReading:
    pitch: float | None = None
    roll: float | None = None
    yaw: float | None = None


@dataclass
class GpsFix:
    lat: float | None = None
    lon: float | None = None
    speed_knots: float | None = None
    heading_deg: float | None = None
    fix_quality: int = 0
    satellites: int = 0
    hdop: float | None = None


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
        self._change_cond = threading.Condition()
        self._version = 0
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
        leak_gpios, leak_labels, leak_combined = _load_leak_meta()
        self.leak_gpios: list[int] = leak_gpios
        self.leak_labels: list[str] = leak_labels
        self.leak_combined: bool = leak_combined
        self.ballast_fore = BallastTankState()
        self.ballast_aft = BallastTankState()
        self.thruster_value: float | None = None
        self.thruster_pwm: int | None = None
        self.esp_status: str = "unknown"
        self.esp_fault: str = "unknown"
        self.last_heartbeat: int | None = None
        self.last_heartbeat_ts: float = 0.0
        self.telemetry_timestamp: float = 0.0

        # Sonar (servo sweep from ESP)
        self.sonar_max_range_m: float = 6.0
        self.sonar_points: dict[int, float | None] = {}
        self.sonar_servo_deg: int | None = None
        self.sonar_connected: bool = False
        self.sonar_last_ts: float = 0.0

        # GPS (USB NMEA on Pi)
        self.gps = GpsFix()
        self.gps_connected: bool = False
        self.gps_device_online: bool = False
        self.gps_status: str = "scanning"  # scanning | online | fix
        self.gps_port: str = ""
        self.gps_track: deque[dict[str, float]] = deque(maxlen=1000)
        self.gps_track_origin: tuple[float, float] | None = None
        self.gps_last_ts: float = 0.0

        # Control — manual default so bench/dashboard starts idle without a gamepad
        self.control_mode: str = "manual"  # xbox | manual | auto
        self.manual_actuators = SubActuators()
        self.auto_actuators = SubActuators()
        self.xbox_actuators = SubActuators()
        self.effective_actuators = SubActuators()

        # YOLO model selection (Auto mode on dashboard)
        self.yolo_model_id: str = ""
        self.yolo_model_status: dict[str, Any] = {
            "state": "idle",
            "error": None,
            "task": None,
            "label": "",
        }

        # ESP diagnostic responses (PING, PINS, TEST)
        self.last_pong_ts: float | None = None
        self.esp_pins_lines: list[str] = []
        self.esp_pins_map: dict[str, str] = {}
        self._capturing_pins: bool = False
        self.last_diagnostic: str = ""
        self.last_diagnostic_ts: float = 0.0

    # ------------------------------------------------------------------
    # Live stream notifications (SSE)
    # ------------------------------------------------------------------

    def _notify(self) -> None:
        with self._change_cond:
            self._version += 1
            self._change_cond.notify_all()

    def get_version(self) -> int:
        with self._change_cond:
            return self._version

    def wait_for_change(self, since: int, timeout: float) -> int:
        deadline = time.monotonic() + max(0.0, timeout)
        with self._change_cond:
            while self._version == since:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    break
                self._change_cond.wait(timeout=remaining)
            return self._version

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
        self._notify()

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
        self._notify()

    def update_gyro(self, pitch: float, roll: float, yaw: float) -> None:
        with self._lock:
            self.gyro = GyroReading(pitch=pitch, roll=roll, yaw=yaw)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_depth(self, depth_m: float) -> None:
        with self._lock:
            self.depth_m = depth_m
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_leaks(self, sensors: list[bool]) -> None:
        with self._lock:
            self.leak_sensors = list(sensors)
            self.leak_triggered = any(sensors)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def _ballast_tank(self, tank: str) -> BallastTankState:
        if tank == "aft":
            return self.ballast_aft
        return self.ballast_fore

    def update_ballast_level(self, level: float, tank: str = "fore") -> None:
        with self._lock:
            t = self._ballast_tank(tank)
            t.level = max(0.0, min(1.0, level))
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_ballast_tank(
        self,
        tank: str,
        level: float | None,
        adc: int | None = None,
        moving: bool | None = None,
        direction: str | None = None,
    ) -> None:
        with self._lock:
            t = self._ballast_tank(tank)
            if level is None or level < 0:
                t.level = None
            else:
                t.level = max(0.0, min(1.0, level))
            if adc is not None:
                t.adc = int(adc)
            if moving is not None:
                t.moving = bool(moving)
            if direction is not None:
                t.direction = str(direction)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_ballast(
        self,
        level: float,
        adc: int | None = None,
        moving: bool | None = None,
        direction: str | None = None,
    ) -> None:
        """Legacy single-tank update (fore)."""
        self.update_ballast_tank("fore", level, adc=adc, moving=moving, direction=direction)

    def update_ballast_cal(
        self,
        tank: str,
        bottom_adc: int | None = None,
        top_adc: int | None = None,
        valid: bool | None = None,
    ) -> None:
        with self._lock:
            t = self._ballast_tank(tank)
            if bottom_adc is not None and bottom_adc >= 0:
                t.cal_bottom_adc = int(bottom_adc)
            if top_adc is not None and top_adc >= 0:
                t.cal_top_adc = int(top_adc)
            if valid is not None:
                t.cal_valid = bool(valid)
            elif t.cal_top_adc is not None and t.cal_bottom_adc is not None:
                span = abs(t.cal_top_adc - t.cal_bottom_adc)
                t.cal_valid = span >= 50
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_thruster(self, value: float, pwm: int) -> None:
        with self._lock:
            self.thruster_value = float(value)
            self.thruster_pwm = int(pwm)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_esp_status(self, status: str) -> None:
        with self._lock:
            self.esp_status = str(status)
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_esp_fault(self, fault: str) -> None:
        with self._lock:
            self.esp_fault = str(fault)
            self.leak_triggered = self.leak_triggered or fault.upper() == "LEAK"
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def update_heartbeat(self, count: int) -> None:
        with self._lock:
            self.last_heartbeat = int(count)
            self.last_heartbeat_ts = time.time()
            self.telemetry_timestamp = time.time()
            self.esp_connected = True
        self._notify()

    def mark_esp_stale(self, stale_after_s: float = 3.0) -> None:
        stale = False
        sonar_stale = False
        with self._lock:
            if self.telemetry_timestamp <= 0:
                pass
            elif time.time() - self.telemetry_timestamp > stale_after_s:
                stale = self.esp_connected
                self.esp_connected = False
            if self.sonar_last_ts > 0 and time.time() - self.sonar_last_ts > stale_after_s:
                sonar_stale = self.sonar_connected
                self.sonar_connected = False
        if stale or sonar_stale:
            self._notify()

    def update_sonar_point(self, angle_deg: int, range_m: float | None) -> None:
        with self._lock:
            self.sonar_points[int(angle_deg)] = range_m
            self.sonar_servo_deg = int(angle_deg)
            self.sonar_connected = True
            self.sonar_last_ts = time.time()
        self._notify()

    def clear_sonar_scan(self) -> None:
        with self._lock:
            self.sonar_points.clear()
        self._notify()

    def update_gps(
        self,
        lat: float,
        lon: float,
        *,
        speed_knots: float | None = None,
        heading_deg: float | None = None,
        fix_quality: int = 0,
        satellites: int = 0,
        hdop: float | None = None,
        port: str = "",
    ) -> None:
        with self._lock:
            self.gps = GpsFix(
                lat=lat,
                lon=lon,
                speed_knots=speed_knots,
                heading_deg=heading_deg,
                fix_quality=fix_quality,
                satellites=satellites,
                hdop=hdop,
            )
            self.gps_connected = True
            self.gps_device_online = True
            self.gps_status = "fix"
            self.gps_last_ts = time.time()
            if port:
                self.gps_port = port
            if self.gps_track_origin is None:
                self.gps_track_origin = (lat, lon)
            self.gps_track.append({
                "lat": lat,
                "lon": lon,
                "ts": time.time(),
            })
        self._notify()

    def update_gps_reception(
        self,
        *,
        satellites: int | None = None,
        hdop: float | None = None,
        fix_quality: int | None = None,
    ) -> None:
        """NMEA is flowing but there may be no position fix yet."""
        with self._lock:
            if satellites is not None:
                self.gps.satellites = satellites
            if hdop is not None:
                self.gps.hdop = hdop
            if fix_quality is not None:
                self.gps.fix_quality = fix_quality
            self.gps_device_online = True
            if not self.gps_connected:
                self.gps_status = "online"
            self.gps_last_ts = time.time()
        self._notify()

    def clear_gps_track(self) -> None:
        with self._lock:
            self.gps_track.clear()
            self.gps_track_origin = None
        self._notify()

    def set_gps_scanning(self) -> None:
        with self._lock:
            self.gps_device_online = False
            self.gps_status = "scanning"
            self.gps_port = ""
        self._notify()

    def set_gps_device(self, port: str) -> None:
        with self._lock:
            self.gps_device_online = True
            self.gps_status = "online" if not self.gps_connected else "fix"
            self.gps_port = port
        self._notify()

    def set_gps_disconnected(self) -> None:
        with self._lock:
            self.gps_device_online = False
            if not self.gps_connected:
                self.gps_status = "scanning"
        self._notify()

    def set_esp_connected(self, connected: bool, port: str = "") -> None:
        with self._lock:
            self.esp_connected = connected
            if port:
                self.esp_port = port
        self._notify()

    def begin_pins_capture(self) -> None:
        with self._lock:
            self._capturing_pins = True
            self.esp_pins_lines = []
            self.esp_pins_map = {}
        self._notify()

    def append_pins_line(self, line: str) -> None:
        notify = False
        with self._lock:
            if not self._capturing_pins:
                return
            self.esp_pins_lines.append(line)
            if line.startswith("PIN "):
                rest = line[4:]
                if "=" in rest:
                    key, val = rest.split("=", 1)
                    self.esp_pins_map[key.strip()] = val.strip()
            notify = True
        if notify:
            self._notify()

    def end_pins_capture(self) -> None:
        with self._lock:
            self._capturing_pins = False
            self.last_diagnostic = "OK PINS"
            self.last_diagnostic_ts = time.time()
        self._notify()

    def is_capturing_pins(self) -> bool:
        with self._lock:
            return self._capturing_pins

    def set_last_pong(self) -> None:
        with self._lock:
            self.last_pong_ts = time.time()
            self.last_diagnostic = "OK PONG"
            self.last_diagnostic_ts = time.time()
        self._notify()

    def set_last_diagnostic(self, line: str) -> None:
        with self._lock:
            self.last_diagnostic = line
            self.last_diagnostic_ts = time.time()
        self._notify()

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
            self.control_mode = mode if mode in ("xbox", "manual", "auto") else "manual"
        self._notify()

    def get_control_mode(self) -> str:
        with self._lock:
            return self.control_mode

    def set_yolo_model_id(self, model_id: str) -> None:
        with self._lock:
            self.yolo_model_id = str(model_id)
        self._notify()

    def get_yolo_model_id(self) -> str:
        with self._lock:
            return self.yolo_model_id

    def set_yolo_model_status(self, status: dict[str, Any]) -> None:
        with self._lock:
            self.yolo_model_status = dict(status)
        self._notify()

    def get_yolo_model_status(self) -> dict[str, Any]:
        with self._lock:
            return dict(self.yolo_model_status)

    def set_ballast_command(
        self, value: float, tank: str = "both"
    ) -> None:
        cmd = max(-1.0, min(1.0, float(value)))
        with self._lock:
            if tank in ("both", "fore"):
                self.ballast_fore.command = cmd
            if tank in ("both", "aft"):
                self.ballast_aft.command = cmd
        self._notify()

    def set_ballast_commands(self, fore: float, aft: float) -> None:
        fore = max(-1.0, min(1.0, float(fore)))
        aft = max(-1.0, min(1.0, float(aft)))
        with self._lock:
            if self.ballast_fore.command == fore and self.ballast_aft.command == aft:
                return
            self.ballast_fore.command = fore
            self.ballast_aft.command = aft
        self._notify()

    def get_ballast_commands(self) -> tuple[float, float]:
        with self._lock:
            return self.ballast_fore.command, self.ballast_aft.command

    def get_ballast_command(self) -> float:
        """Legacy: returns fore command."""
        with self._lock:
            return self.ballast_fore.command

    def set_manual_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            self.manual_actuators = actuators
        self._notify()

    def set_auto_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            self.auto_actuators = actuators
        self._notify()

    def set_xbox_actuators(self, actuators: SubActuators) -> None:
        with self._lock:
            if asdict(self.xbox_actuators) == asdict(actuators):
                return
            self.xbox_actuators = actuators
        self._notify()

    def recompute_effective(self) -> SubActuators:
        changed = False
        with self._lock:
            prev = asdict(self.effective_actuators)
            if self.control_mode == "auto":
                self.effective_actuators = SubActuators(**asdict(self.auto_actuators))
            elif self.control_mode == "manual":
                self.effective_actuators = SubActuators(**asdict(self.manual_actuators))
            elif self.control_mode == "xbox":
                if self.xbox.connected:
                    self.effective_actuators = SubActuators(**asdict(self.xbox_actuators))
                else:
                    self.effective_actuators = SubActuators(**asdict(self.manual_actuators))
            elif self.xbox.connected:
                self.effective_actuators = SubActuators(**asdict(self.xbox_actuators))
            else:
                # Xbox mode selected but pad offline — hold last manual setpoints
                self.effective_actuators = SubActuators(**asdict(self.manual_actuators))
            changed = asdict(self.effective_actuators) != prev
            result = SubActuators(**asdict(self.effective_actuators))
        if changed:
            self._notify()
        return result

    def update_xbox(self, xbox: XboxState) -> None:
        with self._lock:
            self.xbox = xbox
        self._notify()

    # ------------------------------------------------------------------
    # API snapshots
    # ------------------------------------------------------------------

    def telemetry_snapshot(self) -> dict[str, Any]:
        with self._lock:
            age = (
                time.time() - self.telemetry_timestamp
                if self.telemetry_timestamp > 0
                else None
            )
            connected = self.esp_connected and (age is None or age < 3.0)
            return {
                "esp_connected": connected,
                "esp_port": self.esp_port,
                "esp_status": self.esp_status,
                "esp_fault": self.esp_fault,
                "timestamp": self.telemetry_timestamp,
                "telemetry_age_s": age,
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
                    "gpios": self.leak_gpios,
                    "labels": self.leak_labels,
                    "combined": self.leak_combined,
                },
                "ballast": {
                    "fore": self.ballast_fore.as_dict(),
                    "aft": self.ballast_aft.as_dict(),
                },
                "thruster": {
                    "value": self.thruster_value,
                    "pwm": self.thruster_pwm,
                    "connected": self.thruster_pwm is not None,
                },
                "heartbeat": {
                    "count": self.last_heartbeat,
                    "last_ts": self.last_heartbeat_ts,
                },
                "sonar": {
                    "connected": self.sonar_connected,
                    "max_range_m": self.sonar_max_range_m,
                    "servo_deg": self.sonar_servo_deg,
                    "last_ts": self.sonar_last_ts,
                    "points": [
                        {"angle_deg": a, "range_m": r}
                        for a, r in sorted(self.sonar_points.items())
                    ],
                },
                "gps": {
                    "connected": self.gps_connected,
                    "device_online": self.gps_device_online,
                    "status": self.gps_status,
                    "port": self.gps_port,
                    "last_ts": self.gps_last_ts,
                    "lat": self.gps.lat,
                    "lon": self.gps.lon,
                    "speed_knots": self.gps.speed_knots,
                    "heading_deg": self.gps.heading_deg,
                    "fix_quality": self.gps.fix_quality,
                    "satellites": self.gps.satellites,
                    "hdop": self.gps.hdop,
                    "track": list(self.gps_track),
                    "origin": (
                        {"lat": self.gps_track_origin[0], "lon": self.gps_track_origin[1]}
                        if self.gps_track_origin
                        else None
                    ),
                },
            }

    def control_snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "mode": self.control_mode,
                "ballast_fore_command": self.ballast_fore.command,
                "ballast_aft_command": self.ballast_aft.command,
                "ballast_command": self.ballast_fore.command,
                "xbox": self.xbox.as_dict(),
                "effective": self.effective_actuators.as_dict(),
                "auto": self.auto_actuators.as_dict(),
                "manual": self.manual_actuators.as_dict(),
                "xbox_mapped": self.xbox_actuators.as_dict(),
                "timestamp": time.time(),
                "yolo_model_id": self.yolo_model_id,
                "yolo_model_status": dict(self.yolo_model_status),
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
