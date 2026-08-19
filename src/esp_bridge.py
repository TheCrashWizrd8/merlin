"""
esp_bridge.py
-------------
Background ESP32 serial bridge: reads telemetry lines from ESP, sends ballast
and sub actuator commands. Runs in a daemon thread.

Expected ESP → Pi telemetry (sub_rc.ino):
  TEL battery 12.45
  TEL gyro 1.2 -0.5 45.0
  TEL depth 3.45
  TEL leak 0 0 1 0
  TEL ballast fore 0.500 2048 1 FILL
  TEL ballast aft 0.400 1638 0 STOP
  TEL ballastcal fore 500 3500 1
  TEL controls          (+ 7 value lines)
  TEL thruster 0.400 128
  TEL status READY
  TEL fault NONE
  TEL heartbeat 42

Pi → ESP commands:
  B <fore> <aft>\\n                     ballast -1..+1 per tank (ESP drives DIR/PWM on/off)
  CAL B <fore|aft> top|bottom|show
  S2 <y> <z> F <fl> <fr> X <thr>\\n   sub actuators

Ballast hardware (per tank, 5 wires): INA + INB + pot wiper (ADC) + pot 3.3V + pot GND.
Fill/drain is digital only — Makerverse DIR/PWM enable, no motor speed PWM on ballast pumps.
"""

from __future__ import annotations

import json
import glob
import re
import threading
import time
from pathlib import Path
from typing import Callable

import yaml

from src.serial_util import (
    is_esp_usb_device,
    is_gps_usb_device,
    is_usb_serial_port,
    open_serial_port,
)
from src.sub_state import SubActuators, get_sub_state

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"
_TEL_PREFIX = re.compile(r"^TEL\s+", re.IGNORECASE)
_STALE_TELEMETRY_S = 3.0
_BOOT_GRACE_S = 4.0          # ESP32-S3 USB CDC needs time after open/reset
_REOPEN_MIN_INTERVAL_S = 3.0
_USB_WRITE_TIMEOUT_S = 1.0
_ESP_BOOT_LINE = re.compile(
    r"^(ESP-ROM:|rst:|Saved PC:|SPIWP:|mode:|load:|entry |Build:|boot:)",
    re.IGNORECASE,
)


def _resolve_serial_port(configured: str) -> str:
    """Prefer the ESP32 USB by-id path. Never claim a USB GPS dongle."""
    by_id = sorted(glob.glob("/dev/serial/by-id/*"))
    esp_ids = [p for p in by_id if is_esp_usb_device(p) and not is_gps_usb_device(p)]
    if esp_ids:
        return esp_ids[0]

    candidates: list[str] = []
    if configured:
        candidates.append(configured)
    candidates.extend(sorted(glob.glob("/dev/ttyACM*")))
    candidates.extend(sorted(glob.glob("/dev/ttyUSB*")))

    seen: set[str] = set()
    for path in candidates:
        if not path or not Path(path).exists():
            continue
        try:
            real = str(Path(path).resolve())
        except OSError:
            real = path
        if real in seen:
            continue
        seen.add(real)
        if is_gps_usb_device(path):
            continue
        if is_esp_usb_device(path) or path == configured:
            return path
    return configured


def _load_serial_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    sub = cfg.get("sub_serial") or {}
    port = _resolve_serial_port(sub.get("port", "/dev/ttyACM0"))
    return {
        "port": port,
        "baud_rate": int(sub.get("baud_rate", 115200)),
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
    if text == "OK PINS":
        state.begin_pins_capture()
        state.append_pins_line(text)
        return True
    if state.is_capturing_pins():
        if text.startswith("TEL ") or text.startswith("{"):
            state.end_pins_capture()
            return False
        state.append_pins_line(text)
        return True
    if text == "OK PONG":
        if state.is_capturing_pins():
            state.end_pins_capture()
        state.set_last_pong()
        return True
    if text.startswith(("OK TEST", "OK HELP", "OK S2", "ERR TEST", "OK CAL", "ERR CAL")):
        state.set_last_diagnostic(text)
        if text.startswith("OK CAL B "):
            parts = text.split()
            # OK CAL B fore top 1234  |  OK CAL B fore show b t v
            if len(parts) >= 5 and parts[3] in ("top", "bottom"):
                tank = parts[2]
                try:
                    adc = int(parts[4])
                    if parts[3] == "top":
                        state.update_ballast_cal(tank, top_adc=adc)
                    else:
                        state.update_ballast_cal(tank, bottom_adc=adc)
                except ValueError:
                    pass
            elif len(parts) >= 7 and parts[3] == "show":
                tank = parts[2]
                try:
                    state.update_ballast_cal(
                        tank,
                        bottom_adc=int(parts[4]),
                        top_adc=int(parts[5]),
                        valid=bool(int(parts[6])),
                    )
                except ValueError:
                    pass
            elif len(parts) >= 4 and parts[2] in ("top", "bottom"):
                try:
                    adc = int(parts[3])
                    if parts[2] == "top":
                        state.update_ballast_cal("fore", top_adc=adc)
                    else:
                        state.update_ballast_cal("fore", bottom_adc=adc)
                except ValueError:
                    pass
        return True
    if text.startswith("sub_rc ready"):
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

    if line.startswith("{"):
        try:
            obj = json.loads(line)
            return _parse_json_telemetry(obj, state)
        except json.JSONDecodeError:
            return False

    if not _TEL_PREFIX.match(line):
        return False

    parts = line.split()
    if len(parts) < 3:
        return False

    kind = parts[1].lower()
    try:
        if kind == "battery":
            state.update_battery(float(parts[2]))
            return True
        if kind == "gyro" and len(parts) >= 5:
            state.update_gyro(float(parts[2]), float(parts[3]), float(parts[4]))
            return True
        if kind == "depth":
            state.update_depth(float(parts[2]))
            return True
        if kind == "leak" and len(parts) >= 6:
            sensors = [bool(int(x)) for x in parts[2:6]]
            state.update_leaks(sensors)
            return True
        if kind == "ballastcal" and len(parts) >= 6:
            state.update_ballast_cal(
                parts[2],
                bottom_adc=int(parts[3]),
                top_adc=int(parts[4]),
                valid=bool(int(parts[5])),
            )
            return True
        if kind == "ballast" and len(parts) >= 7 and parts[2] in ("fore", "aft"):
            raw_level = float(parts[3])
            adc = int(parts[4])
            # -1 level with adc >= 0 means unwired sentinel; otherwise use raw/cal level
            level = None if (raw_level < 0 and adc < 0) else max(0.0, raw_level)
            state.update_ballast_tank(
                parts[2],
                level,
                adc=int(parts[4]),
                moving=bool(int(parts[5])),
                direction=parts[6],
            )
            return True
        if kind == "ballast" and len(parts) >= 3:
            level = float(parts[2])
            adc = int(parts[3]) if len(parts) >= 4 else None
            moving = bool(int(parts[4])) if len(parts) >= 5 else None
            direction = parts[5] if len(parts) >= 6 else None
            state.update_ballast_tank(
                "fore", level, adc=adc, moving=moving, direction=direction
            )
            return True
        if kind == "thruster" and len(parts) >= 4:
            state.update_thruster(float(parts[2]), int(parts[3]))
            return True
        if kind == "status":
            state.update_esp_status(" ".join(parts[2:]))
            return True
        if kind == "fault":
            state.update_esp_fault(" ".join(parts[2:]))
            return True
        if kind == "heartbeat":
            state.update_heartbeat(int(parts[2]))
            return True
        if kind == "sonarpt" and len(parts) >= 4:
            angle = int(float(parts[2]))
            raw = float(parts[3])
            range_m = None if raw < 0 else raw
            state.update_sonar_point(angle, range_m)
            return True
        if kind == "sonar" and len(parts) >= 3 and parts[2].lower() == "sweep":
            state.clear_sonar_scan()
            return True
        if kind == "controls":
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
            state.update_ballast(float(obj["level"]))
            return True
    except (KeyError, TypeError, ValueError):
        return False
    return False


def format_ballast_command(fore: float, aft: float) -> str:
    f = max(-1.0, min(1.0, float(fore)))
    a = max(-1.0, min(1.0, float(aft)))
    return f"B {f:.3f} {a:.3f}\n"


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
        self._configured_port = port or cfg["port"]
        self.port = _resolve_serial_port(self._configured_port)
        self.baud_rate = baud_rate or cfg["baud_rate"]
        self.send_interval = 1.0 / send_hz
        self._on_line = on_line
        self._serial = None
        self._ser_lock = threading.Lock()
        self._opened_at = 0.0
        self._last_close_at = 0.0
        self._stop = threading.Event()
        self._reader: threading.Thread | None = None
        self._writer: threading.Thread | None = None
        self._stale: threading.Thread | None = None
        self._state = get_sub_state()
        self._diag_mode = False
        self._diag_timer: threading.Timer | None = None
        self._skip_lines = 0
        self._last_tx_s2 = ""
        self._last_tx_b = ""

    def set_diag_mode(self, enabled: bool, resume_after_s: float | None = None) -> None:
        if self._diag_timer is not None:
            self._diag_timer.cancel()
            self._diag_timer = None
        self._diag_mode = enabled
        if enabled and resume_after_s and resume_after_s > 0:
            self._diag_timer = threading.Timer(resume_after_s, self._resume_diag)
            self._diag_timer.daemon = True
            self._diag_timer.start()

    def _resume_diag(self) -> None:
        self._diag_mode = False
        self._diag_timer = None

    def _open(self) -> bool:
        with self._ser_lock:
            if self._serial is not None and self._serial.is_open:
                return True
            since_close = time.time() - self._last_close_at
            if since_close < _REOPEN_MIN_INTERVAL_S:
                return False
            self.port = _resolve_serial_port(self._configured_port)
            if not self.port or is_gps_usb_device(self.port):
                self._state.set_esp_connected(False, self.port)
                return False
            try:
                self._serial = open_serial_port(
                    self.port,
                    self.baud_rate,
                    timeout=0.05,
                    write_timeout=_USB_WRITE_TIMEOUT_S
                    if is_usb_serial_port(self.port)
                    else 0.25,
                )
                self._opened_at = time.time()
                self._state.set_esp_connected(True, self.port)
                link = (
                    f"USB serial ({self.port})"
                    if is_usb_serial_port(self.port)
                    else "GPIO UART (14 TX / 15 RX)"
                    if "serial0" in self.port.lower() or "ttyama" in self.port.lower()
                    else self.port
                )
                self._state.append_serial("sys", f"Opened {link} @ {self.baud_rate}")
                return True
            except Exception as exc:
                self._serial = None
                self._state.set_esp_connected(False, self.port)
                self._state.append_serial("sys", f"Serial open failed: {exc}")
                return False

    def _ready_for_write(self) -> bool:
        if self._opened_at <= 0:
            return False
        return (time.time() - self._opened_at) >= _BOOT_GRACE_S

    def _should_log_rx(self, text: str) -> bool:
        if _ESP_BOOT_LINE.match(text):
            return False
        if text.startswith("CHK") and len(text) < 24:
            return False
        return True

    def _reader_loop(self) -> None:
        while not self._stop.is_set():
            if not self._open():
                time.sleep(1.0)
                continue
            try:
                with self._ser_lock:
                    ser = self._serial
                    if ser is None or not ser.is_open:
                        continue
                    raw = ser.readline()
            except Exception as exc:
                self._state.append_serial("sys", f"Read error: {exc}")
                self._close_serial()
                time.sleep(_REOPEN_MIN_INTERVAL_S)
                continue
            if raw is None or not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not text:
                continue

            if self._skip_lines > 0:
                self._skip_lines -= 1
                continue

            if self._state.is_capturing_pins() and text.startswith("TEL "):
                self._state.end_pins_capture()

            if self._should_log_rx(text):
                self._state.append_serial("rx", text)
            if self._on_line:
                self._on_line(text)

            if parse_diagnostic_line(text, self._state):
                continue

            if text.startswith("TEL controls"):
                self._skip_lines = 7
                continue

            parse_telemetry_line(text, self._state)

    def _writer_loop(self) -> None:
        while not self._stop.is_set():
            if self._diag_mode:
                time.sleep(self.send_interval)
                continue
            if not self._open():
                time.sleep(self.send_interval)
                continue
            if not self._ready_for_write():
                time.sleep(0.25)
                continue
            act = self._state.recompute_effective()
            fore_cmd, aft_cmd = self._state.get_ballast_commands()
            ballast_line = format_ballast_command(fore_cmd, aft_cmd)
            s2_line = format_actuator_command(act)
            lines = [ballast_line, s2_line]
            write_failed = False
            for line in lines:
                try:
                    with self._ser_lock:
                        ser = self._serial
                        if ser is None or not ser.is_open:
                            write_failed = True
                            break
                        ser.write(line.encode("ascii"))
                        ser.flush()
                    if line.startswith("S2"):
                        if line != self._last_tx_s2:
                            self._state.append_serial("tx", line.rstrip("\n"))
                            self._last_tx_s2 = line
                    elif line.startswith("B"):
                        if line != self._last_tx_b:
                            self._state.append_serial("tx", line.rstrip("\n"))
                            self._last_tx_b = line
                    else:
                        self._state.append_serial("tx", line.rstrip("\n"))
                except Exception as exc:
                    self._state.append_serial("sys", f"Write error: {exc}")
                    write_failed = True
                    break
            if write_failed:
                self._close_serial()
                time.sleep(_REOPEN_MIN_INTERVAL_S)
            time.sleep(self.send_interval)

    def _stale_loop(self) -> None:
        while not self._stop.is_set():
            self._state.mark_esp_stale(_STALE_TELEMETRY_S)
            time.sleep(1.0)

    def _close_serial(self) -> None:
        with self._ser_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None
            self._last_close_at = time.time()
        self._state.set_esp_connected(False, self.port)

    def start(self) -> None:
        if self._reader and self._reader.is_alive():
            return
        self._stop.clear()
        self._reader = threading.Thread(target=self._reader_loop, daemon=True, name="esp-rx")
        self._writer = threading.Thread(target=self._writer_loop, daemon=True, name="esp-tx")
        self._stale = threading.Thread(target=self._stale_loop, daemon=True, name="esp-stale")
        self._reader.start()
        self._writer.start()
        self._stale.start()

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
            with self._ser_lock:
                ser = self._serial
                if ser is None or not ser.is_open:
                    return False
                ser.write(line.encode("ascii"))
                ser.flush()
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
