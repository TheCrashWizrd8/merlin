"""
esp_bridge.py
-------------
Background ESP32 serial bridge: reads telemetry lines from ESP, sends ballast
and sub actuator commands. Runs in a daemon thread.

Expected ESP → Pi telemetry (one line each):
  TEL battery 12.45
  TEL gyro 1.2 -0.5 45.0
  TEL depth 3.45
  TEL leak 0 0 1 0
  TEL ballast 0.5

Also accepts JSON:
  {"t":"battery","v":12.45}
  {"t":"gyro","pitch":1.2,"roll":-0.5,"yaw":45.0}
  {"t":"depth","m":3.45}
  {"t":"leak","sensors":[0,0,1,0]}
  {"t":"ballast","level":0.5}

Pi → ESP commands:
  B <value>\\n                         ballast -1..+1
  S2 <y> <z> F <fl> <fr> X <thr>\\n   sub actuators
"""

from __future__ import annotations

import json
import re
import threading
import time
from pathlib import Path
from typing import Callable

import yaml

from src.sub_state import SubActuators, get_sub_state
from src.serial_util import open_serial_port

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"

_TEL_PREFIX = re.compile(r"^TEL\s+", re.IGNORECASE)


def _load_serial_config() -> dict:
    with open(CONFIG_PATH) as f:
        cfg = yaml.safe_load(f) or {}
    ser = cfg.get("serial") or {}
    return {
        "port": ser.get("port", "/dev/ttyACM0"),
        "baud_rate": int(ser.get("baud_rate", 115200)),
    }


def parse_diagnostic_line(line: str, state=None) -> bool:
    """Parse ESP diagnostic replies (PING/PINS/TEST). Returns True if handled."""
    if state is None:
        state = get_sub_state()

    text = line.strip()
    if not text:
        return False

    if text == "OK PINS BEGIN":
        state.begin_pins_capture()
        return True
    if text == "OK PINS END":
        state.end_pins_capture()
        return True
    if state.is_capturing_pins():
        state.append_pins_line(text)
        return True
    if text == "OK PONG":
        state.set_last_pong()
        return True
    if text.startswith(("OK TEST", "OK LEAK", "OK ADC", "OK HELP")):
        state.set_last_diagnostic(text)
        return True
    return False


def parse_telemetry_line(line: str, state=None) -> bool:
    """Parse one ESP telemetry line. Returns True if recognised."""
    if state is None:
        state = get_sub_state()

    line = line.strip()
    if not line:
        return False

    # JSON telemetry
    if line.startswith("{"):
        try:
            obj = json.loads(line)
            return _parse_json_telemetry(obj, state)
        except json.JSONDecodeError:
            return False

    # Text telemetry: TEL <type> ...
    if _TEL_PREFIX.match(line):
        parts = line.split()
        if len(parts) < 3:
            return False
        kind = parts[1].lower()
        try:
            if kind == "battery" and len(parts) >= 3:
                state.update_battery(float(parts[2]))
                return True
            if kind == "gyro" and len(parts) >= 5:
                state.update_gyro(float(parts[2]), float(parts[3]), float(parts[4]))
                return True
            if kind == "depth" and len(parts) >= 3:
                state.update_depth(float(parts[2]))
                return True
            if kind == "leak" and len(parts) >= 3:
                sensors = [bool(int(x)) for x in parts[2:]]
                state.update_leaks(sensors)
                return True
            if kind == "ballast" and len(parts) >= 3:
                state.update_ballast_level(float(parts[2]))
                return True
        except (ValueError, IndexError):
            return False

    return False


def _parse_json_telemetry(obj: dict, state) -> bool:
    t = str(obj.get("t", "")).lower()
    try:
        if t == "battery":
            state.update_battery(float(obj["v"]))
            return True
        if t == "gyro":
            state.update_gyro(float(obj["pitch"]), float(obj["roll"]), float(obj["yaw"]))
            return True
        if t == "depth":
            state.update_depth(float(obj["m"]))
            return True
        if t == "leak":
            sensors = [bool(x) for x in obj.get("sensors", [])]
            state.update_leaks(sensors)
            return True
        if t == "ballast":
            state.update_ballast_level(float(obj["level"]))
            return True
    except (KeyError, TypeError, ValueError):
        return False
    return False


def format_ballast_command(value: float) -> str:
    return f"B {value:.3f}\n"


def format_actuator_command(act: SubActuators) -> str:
    return (
        f"S2 {act.aft_steer_y:.3f} {act.aft_steer_z:.3f} "
        f"F {act.fin_left:.3f} {act.fin_right:.3f} "
        f"X {act.thruster_x:.3f}\n"
    )


class EspBridge:
    """Read ESP telemetry and write control commands over serial."""

    def __init__(
        self,
        port: str | None = None,
        baud_rate: int | None = None,
        send_hz: float = 20.0,
        on_line: Callable[[str], None] | None = None,
    ) -> None:
        cfg = _load_serial_config()
        self.port = port or cfg["port"]
        self.baud_rate = baud_rate or cfg["baud_rate"]
        self.send_interval = 1.0 / send_hz
        self._on_line = on_line
        self._serial = None
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._writer: threading.Thread | None = None
        self._state = get_sub_state()

    def _open(self) -> bool:
        if self._serial is not None:
            return True
        try:
            self._serial = open_serial_port(
                self.port,
                self.baud_rate,
                timeout=0.05,
                write_timeout=0.1,
            )
            self._state.set_esp_connected(True, self.port)
            link = "GPIO UART (14 TX / 15 RX)" if "serial0" in self.port.lower() or "ttyama" in self.port.lower() else self.port
            self._state.append_serial("sys", f"Opened {link} @ {self.baud_rate}")
            return True
        except Exception as exc:
            self._state.set_esp_connected(False, self.port)
            self._state.append_serial("sys", f"Serial open failed: {exc}")
            return False

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            if not self._open():
                time.sleep(2.0)
                continue
            try:
                raw = self._serial.readline()
            except Exception as exc:
                self._state.append_serial("sys", f"Read error: {exc}")
                self._close_serial()
                time.sleep(1.0)
                continue
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not text:
                continue
            self._state.append_serial("rx", text)
            if self._on_line:
                self._on_line(text)
            if parse_diagnostic_line(text, self._state):
                continue
            parse_telemetry_line(text, self._state)

    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            if not self._open():
                time.sleep(self.send_interval)
                continue
            act = self._state.recompute_effective()
            ballast = self._state.get_ballast_command()
            lines = [format_ballast_command(ballast), format_actuator_command(act)]
            for line in lines:
                try:
                    self._serial.write(line.encode("ascii"))
                    self._serial.flush()
                    self._state.append_serial("tx", line.rstrip("\n"))
                except Exception as exc:
                    self._state.append_serial("sys", f"Write error: {exc}")
                    self._close_serial()
                    break
            time.sleep(self.send_interval)

    def _close_serial(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception:
                pass
            self._serial = None
        self._state.set_esp_connected(False, self.port)

    def start(self) -> None:
        if self._reader and self._reader.is_alive():
            return
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True, name="esp-rx")
        self._writer = threading.Thread(target=self._writer_loop, daemon=True, name="esp-tx")
        self._reader.start()
        self._writer.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_serial()

    def send_raw(self, line: str) -> bool:
        """Send a raw line to ESP (for manual serial monitor POST)."""
        if not self._open():
            return False
        if not line.endswith("\n"):
            line += "\n"
        try:
            self._serial.write(line.encode("ascii"))
            self._serial.flush()
            self._state.append_serial("tx", line.rstrip("\n"))
            return True
        except Exception as exc:
            self._state.append_serial("sys", f"Raw send failed: {exc}")
            self._close_serial()
            return False


_bridge: EspBridge | None = None
_bridge_lock = threading.Lock()


def get_esp_bridge(
    port: str | None = None,
    baud_rate: int | None = None,
    autostart: bool = False,
) -> EspBridge:
    global _bridge
    with _bridge_lock:
        if _bridge is None:
            _bridge = EspBridge(port=port, baud_rate=baud_rate)
            if autostart:
                _bridge.start()
        return _bridge
