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
    is_gpio_uart_port,
    is_usb_serial_port,
    open_serial_port,
)
from src.servo_center import (
    centers_snapshot,
    format_center_command,
    format_tune_command,
    format_tune_pwm_command,
    format_travel_tune_command,
    sync_all_servo_cal_commands,
    wire_s2_actuators,
    write_servo_center_tick,
)
from src.sub_state import SubActuators, get_sub_state

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"
PINS_PATH = Path(__file__).parent.parent / "config" / "pins.yaml"
_fin_right_invert: bool | None = None
_TEL_PREFIX = re.compile(r"^TEL\s+", re.IGNORECASE)
_STALE_TELEMETRY_S = 3.0
_BOOT_GRACE_S = 4.0          # ESP32-S3 USB CDC needs time after open/reset
_REOPEN_MIN_INTERVAL_S = 3.0
_USB_WRITE_TIMEOUT_S = 1.0
_KEEPALIVE_S = 1.0           # re-send last S2/B so ESP serial timeout stays armed
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
        if is_esp_usb_device(path):
            return path
        if is_gpio_uart_port(path) and path == configured:
            return path
        if path == configured and is_usb_serial_port(path):
            # Configured ACM/USB path exists but is not Espressif by-id — still try
            # (some boards omit by-id); GPS-like devices are skipped above.
            return path
    if configured and Path(configured).exists() and not is_gps_usb_device(configured):
        return configured
    return configured


_ballast_swap_motors: bool | None = None


def ballast_swap_motor_commands_enabled() -> bool:
    global _ballast_swap_motors
    if _ballast_swap_motors is not None:
        return _ballast_swap_motors
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    ballast = cfg.get("ballast") or {}
    _ballast_swap_motors = bool(ballast.get("swap_motor_commands", False))
    return _ballast_swap_motors


def wire_ballast_commands_for_esp(fore: float, aft: float) -> tuple[float, float]:
    """Map Pi/dashboard tank names to ESP B-line fore/aft motor channels."""
    f = float(fore)
    a = float(aft)
    if ballast_swap_motor_commands_enabled():
        return a, f
    return f, a


def transform_ballast_motor_line(line: str) -> str:
    """Swap fore/aft in B / TEST B motor lines when swap_motor_commands is set."""
    if not ballast_swap_motor_commands_enabled():
        return line
    text = line.strip()
    parts = text.split()
    if len(parts) >= 3 and parts[0] == "B":
        try:
            fore = float(parts[1])
            aft = float(parts[2])
        except ValueError:
            return line
        w_fore, w_aft = wire_ballast_commands_for_esp(fore, aft)
        return f"B {w_fore:.3f} {w_aft:.3f}"
    if len(parts) >= 4 and parts[0] == "TEST" and parts[1] == "B":
        tank = parts[2].lower()
        if tank in ("fore", "aft"):
            parts[2] = "aft" if tank == "fore" else "fore"
            return " ".join(parts)
    if len(parts) == 3 and parts[0] == "TEST" and parts[1] == "B":
        # TEST B fill|drain|stop → both tanks
        return line
    return line


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
    if text.startswith("OK ESC") or text.startswith("ERR ESC"):
        state.note_esc_line(text)
        state.set_last_diagnostic(text)
        return True
    if text.startswith(("OK TEST", "OK HELP", "OK S2", "OK B", "ERR TEST", "OK CAL", "ERR CAL")):
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
            elif len(parts) >= 4 and parts[3] in ("clear", "reset"):
                state.clear_ballast_cal(parts[2])
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


def _update_gyro(state, pitch: float, roll: float, yaw: float) -> None:
    from src.imu_attitude import map_reported_attitude

    pitch, roll = map_reported_attitude(pitch, roll)
    state.update_gyro(pitch, roll, yaw)


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
            _update_gyro(state, float(parts[2]), float(parts[3]), float(parts[4]))
            return True
        if kind == "depth":
            state.update_depth(float(parts[2]))
            return True
        if kind == "leak" and len(parts) >= 6:
            sensors = [bool(int(x)) for x in parts[2:6]]
            state.update_leaks(sensors)
            from src.fault_indicator import notify_tel_leak

            notify_tel_leak(sensors)
            return True
        if kind == "ballastcal" and len(parts) >= 6:
            bottom = int(parts[3])
            top = int(parts[4])
            valid = bool(int(parts[5]))
            if bottom < 0 and top < 0 and not valid:
                state.clear_ballast_cal(parts[2])
            else:
                state.update_ballast_cal(
                    parts[2],
                    bottom_adc=bottom,
                    top_adc=top,
                    valid=valid,
                )
            return True
        if kind == "ballast" and len(parts) >= 7 and parts[2] in ("fore", "aft"):
            raw_level = float(parts[3])
            adc = int(parts[4])
            # ESP sends level -1 when pot GPIO unwired; ignore stale ADC on that path.
            level = None if raw_level < 0 else max(0.0, min(1.0, raw_level))
            mv_milli: int | None = None
            moving_idx = 5
            direction_idx = 6
            if len(parts) >= 8:
                mv_milli = int(parts[5])
                moving_idx = 6
                direction_idx = 7
            state.update_ballast_tank(
                parts[2],
                level,
                adc=adc,
                mv_milli=mv_milli,
                moving=bool(int(parts[moving_idx])),
                direction=parts[direction_idx],
            )
            return True
        if kind == "ballast" and len(parts) >= 3:
            if parts[2] in ("fore", "aft"):
                return False
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
        if kind == "esc" and len(parts) >= 4:
            ready = bool(int(parts[2]))
            neutral_ms = int(parts[3])
            hold_ms = int(parts[4]) if len(parts) >= 5 else None
            pulse_us = int(parts[5]) if len(parts) >= 6 else None
            state.update_esc(
                ready, neutral_ms, arm_hold_ms=hold_ms, pulse_us=pulse_us
            )
            return True
        if kind == "pca9685" and len(parts) >= 6:
            state.update_pca9685(
                bool(int(parts[2])),
                int(parts[3], 16),
                int(parts[4]),
                int(parts[5]),
            )
            return True
        if kind == "status":
            state.update_esp_status(" ".join(parts[2:]))
            return True
        if kind == "fault":
            fault = " ".join(parts[2:])
            state.update_esp_fault(fault)
            from src.fault_indicator import notify_tel_fault

            notify_tel_fault(fault)
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
            _update_gyro(state, float(obj["pitch"]), float(obj["roll"]), float(obj["yaw"]))
            return True
        if t == "depth":
            state.update_depth(float(obj["m"]))
            return True
        if t == "leak":
            sensors = [bool(x) for x in obj.get("sensors", [])]
            state.update_leaks(sensors)
            from src.fault_indicator import notify_tel_leak

            notify_tel_leak(sensors)
            return True
        if t == "ballast":
            state.update_ballast(float(obj["level"]))
            return True
    except (KeyError, TypeError, ValueError):
        return False
    return False


def format_ballast_command(fore: float, aft: float) -> str:
    f, a = wire_ballast_commands_for_esp(fore, aft)
    f = max(-1.0, min(1.0, f))
    a = max(-1.0, min(1.0, a))
    return f"B {f:.3f} {a:.3f}\n"


def fin_right_invert_enabled() -> bool:
    global _fin_right_invert
    if _fin_right_invert is not None:
        return _fin_right_invert
    try:
        with open(PINS_PATH) as f:
            data = yaml.safe_load(f) or {}
        fin5 = ((data.get("pca9685") or {}).get("channels") or {}).get("fin_5") or {}
        _fin_right_invert = bool(fin5.get("invert_command", False))
    except OSError:
        _fin_right_invert = False
    return _fin_right_invert


_THRUSTER_TX_DEADBAND = 0.08
_SERVO_TX_DEADBAND = 0.02


def _snap_thruster_for_tx(value: float) -> float:
    v = float(value)
    if abs(v) < _THRUSTER_TX_DEADBAND:
        return 0.0
    return max(-1.0, min(1.0, v))


def _snap_servo_for_tx(value: float) -> float:
    """Collapse near-zero fin/steer commands so the ESP holds one centre pulse."""
    v = float(value)
    if abs(v) < _SERVO_TX_DEADBAND:
        return 0.0
    return max(-1.0, min(1.0, v))


def format_actuator_command(act: SubActuators) -> str:
    rested = SubActuators(
        aft_steer_y=_snap_servo_for_tx(act.aft_steer_y),
        aft_steer_z=_snap_servo_for_tx(act.aft_steer_z),
        thruster_x=act.thruster_x,
        fin_left=_snap_servo_for_tx(act.fin_left),
        fin_right=_snap_servo_for_tx(act.fin_right),
    )
    wire = wire_s2_actuators(rested, invert_fin_r=fin_right_invert_enabled())
    thr = _snap_thruster_for_tx(wire.thruster_x)
    return (
        f"S2 {_snap_servo_for_tx(wire.aft_steer_y):.3f} {_snap_servo_for_tx(wire.aft_steer_z):.3f} "
        f"F {_snap_servo_for_tx(wire.fin_left):.3f} {_snap_servo_for_tx(wire.fin_right):.3f} "
        f"X {thr:.3f}\n"
    )


def sonar_sweep_enabled() -> bool:
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        return False
    return bool((cfg.get("sonar") or {}).get("sweep_enabled", False))


def format_sonar_command(enabled: bool) -> str:
    return f"SONAR {1 if enabled else 0}\n"


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
        self._last_tx_any = 0.0
        self._center_sync_pending = True
        self._clear_servo_cal_pending = True

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
                self._center_sync_pending = True
                self._clear_servo_cal_pending = True
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
            if self._center_sync_pending:
                try:
                    with self._ser_lock:
                        ser = self._serial
                        if ser is not None and ser.is_open:
                            for cmd in sync_all_servo_cal_commands():
                                line = cmd + "\n"
                                ser.write(line.encode("ascii"))
                            ser.write(format_sonar_command(sonar_sweep_enabled()).encode("ascii"))
                            ser.flush()
                    self._last_tx_any = time.time()
                    self._center_sync_pending = False
                    for cmd in sync_all_servo_cal_commands():
                        self._state.append_serial("tx", cmd)
                except Exception as exc:
                    self._state.append_serial("sys", f"Servo center sync failed: {exc}")
            if self._clear_servo_cal_pending:
                try:
                    with self._ser_lock:
                        ser = self._serial
                        if ser is not None and ser.is_open:
                            ser.write(b"CAL S DONE\n")
                            ser.flush()
                    self._clear_servo_cal_pending = False
                    self._state.set_servo_cal_active(False)
                    self._state.append_serial("tx", "CAL S DONE")
                except Exception as exc:
                    self._state.append_serial("sys", f"Servo cal clear failed: {exc}")
            act = self._state.recompute_effective()
            fore_cmd, aft_cmd = self._state.get_ballast_commands()
            ballast_line = format_ballast_command(fore_cmd, aft_cmd)
            s2_line = format_actuator_command(act)
            now = time.time()
            s2_changed = s2_line != self._last_tx_s2
            b_changed = ballast_line != self._last_tx_b
            keepalive = (now - self._last_tx_any) >= _KEEPALIVE_S
            if not s2_changed and not b_changed and not keepalive:
                time.sleep(self.send_interval)
                continue
            lines = []
            if b_changed or keepalive:
                lines.append(ballast_line)
            if s2_changed or keepalive:
                lines.append(s2_line)
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
                    self._last_tx_any = now
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
        self._state.set_servo_cal_active(False)
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
        line = transform_ballast_motor_line(line)
        if not line.endswith("\n"):
            line += "\n"
        deadline = time.time() + 8.0
        while not self._ready_for_write() and time.time() < deadline:
            time.sleep(0.05)
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

    def sync_servo_center(self) -> bool:
        ok = True
        for cmd in sync_all_servo_cal_commands():
            if not self.send_raw(cmd):
                ok = False
        return ok

    def tune_servo_center(self, channel: int, tick: int) -> bool:
        if self.send_raw(format_tune_command(channel, tick)):
            return True
        return self.send_raw(format_tune_pwm_command(channel, tick))

    def tune_servo_travel(self, channel: int, end: str, tick: int) -> bool:
        try:
            cmd = format_travel_tune_command(channel, end, tick)
        except ValueError:
            return False
        if self.send_raw(cmd):
            return True
        return self.send_raw(format_tune_pwm_command(channel, tick))


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
