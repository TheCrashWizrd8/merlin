"""
gps_reader.py
-------------
Background USB GPS reader (NMEA). Auto-detects the GPS serial port, probes
baud rates, and reconnects when the dongle is plugged in later.
"""

from __future__ import annotations

import glob
import math
import threading
import time
from pathlib import Path

import yaml

from src.serial_util import open_serial_port
from src.sub_state import get_sub_state

CONFIG_PATH = Path(__file__).parent.parent / "config" / "hardware.yaml"
_ESP_SKIP_PATTERNS = ("espressif", "esp32", "usb jtag", "usb-jtag")
_GPS_HINT_PATTERNS = (
    "gps", "gnss", "u-blox", "ublox", "navis", "vk-172", "vk172",
    "neo-6", "neo-7", "neo-m", "globalSat", "globalsat", "bu-353",
)
_DEVICE_STALE_S = 10.0
_PROBE_S = 1.2
_DEFAULT_BAUD_RATES = (9600, 115200, 38400, 4800, 57600)


def _load_gps_config() -> dict:
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except OSError:
        cfg = {}
    gps = cfg.get("gps") or {}
    port = gps.get("port")
    baud_rates = gps.get("baud_rates") or list(_DEFAULT_BAUD_RATES)
    return {
        "enabled": bool(gps.get("enabled", True)),
        "port": str(port).strip() if port else "",
        "baud_rates": [int(b) for b in baud_rates],
        "scan_interval_s": float(gps.get("scan_interval_s", 3.0)),
        "track_max_points": int(gps.get("track_max_points", 1000)),
    }


def _realpath(port: str) -> str:
    try:
        return str(Path(port).resolve())
    except OSError:
        return port


def _is_esp_by_id(path: str) -> bool:
    low = path.lower()
    return any(p in low for p in _ESP_SKIP_PATTERNS)


def _looks_like_nmea(line: bytes) -> bool:
    text = line.strip()
    return bool(text.startswith(b"$G") and b"," in text)


def list_gps_candidates(*, esp_port: str = "", prefer_port: str = "") -> list[str]:
    """Rank serial devices most likely to be a USB GPS module."""
    esp_real = _realpath(esp_port) if esp_port else ""
    prefer_real = _realpath(prefer_port) if prefer_port else ""
    ranked: list[tuple[int, str]] = []
    seen: set[str] = set()

    def add(path: str, score: int) -> None:
        if not path or not Path(path).exists():
            return
        real = _realpath(path)
        if real in seen:
            return
        if esp_real and real == esp_real:
            return
        seen.add(real)
        ranked.append((score, path))

    if prefer_port:
        add(prefer_port, 200)

    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        if _is_esp_by_id(path):
            continue
        low = path.lower()
        score = 20
        if any(h in low for h in _GPS_HINT_PATTERNS):
            score = 120
        if prefer_real and _realpath(path) == prefer_real:
            score = 200
        add(path, score)

    for pattern in ("/dev/ttyUSB*", "/dev/ttyACM*"):
        for path in sorted(glob.glob(pattern)):
            score = 10
            if prefer_real and _realpath(path) == prefer_real:
                score = 200
            add(path, score)

    ranked.sort(key=lambda item: (-item[0], item[1]))
    return [path for _, path in ranked]


def probe_nmea_port(port: str, baud_rates: list[int], probe_s: float = _PROBE_S) -> int | None:
    """Open port briefly and return the baud rate if NMEA is seen."""
    for baud in baud_rates:
        ser = None
        try:
            ser = open_serial_port(port, baud, timeout=0.15, write_timeout=0.15)
            try:
                ser.reset_input_buffer()
            except Exception:
                pass
            buf = b""
            start = time.monotonic()
            while time.monotonic() - start < probe_s:
                chunk = ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if _looks_like_nmea(line):
                            return baud
                else:
                    time.sleep(0.05)
        except Exception:
            continue
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
    return None


def discover_gps_port(
    *,
    esp_port: str = "",
    prefer_port: str = "",
    baud_rates: list[int] | None = None,
) -> tuple[str, int] | None:
    """Scan serial ports and return (path, baud) for the first NMEA source."""
    rates = baud_rates or list(_DEFAULT_BAUD_RATES)
    for candidate in list_gps_candidates(esp_port=esp_port, prefer_port=prefer_port):
        baud = probe_nmea_port(candidate, rates)
        if baud is not None:
            return candidate, baud
    return None


def _nmea_deg(value: str, direction: str) -> float | None:
    if not value or not direction:
        return None
    try:
        dot = value.index(".")
        deg = float(value[: dot - 2])
        minutes = float(value[dot - 2 :])
        result = deg + minutes / 60.0
        if direction.upper() in ("S", "W"):
            result = -result
        return result
    except (ValueError, IndexError):
        return None


def parse_nmea_line(line: str, state=None) -> bool:
    """Parse one NMEA sentence. Returns True if a fix was applied."""
    if state is None:
        state = get_sub_state()

    text = line.strip()
    if not text.startswith("$"):
        return False

    parts = text.split(",")
    if len(parts) < 2:
        return False

    sentence = parts[0]
    if sentence.endswith("GGA") and len(parts) >= 10:
        quality = 0
        sats = 0
        hdop = None
        try:
            quality = int(parts[6] or "0")
            sats = int(parts[7] or "0")
            if parts[8]:
                hdop = float(parts[8])
        except ValueError:
            pass
        if quality <= 0:
            return False
        lat = _nmea_deg(parts[2], parts[3])
        lon = _nmea_deg(parts[4], parts[5])
        if lat is None or lon is None:
            return False
        state.update_gps(
            lat,
            lon,
            fix_quality=quality,
            satellites=sats,
            hdop=hdop,
        )
        return True

    if sentence.endswith("RMC") and len(parts) >= 10:
        if parts[2] != "A":
            return False
        lat = _nmea_deg(parts[3], parts[4])
        lon = _nmea_deg(parts[5], parts[6])
        if lat is None or lon is None:
            return False
        speed = None
        heading = None
        try:
            if parts[7]:
                speed = float(parts[7])
            if parts[8]:
                heading = float(parts[8])
        except ValueError:
            pass
        state.update_gps(lat, lon, speed_knots=speed, heading_deg=heading, fix_quality=1)
        return True

    return False


class GpsReader:
    """Auto-detect and read NMEA from a USB GPS module."""

    def __init__(self, *, esp_port: str = "", port: str | None = None) -> None:
        cfg = _load_gps_config()
        self._enabled = cfg["enabled"]
        self._prefer_port = port or cfg["port"]
        self._baud_rates = cfg["baud_rates"]
        self._scan_interval = cfg["scan_interval_s"]
        self._esp_port = esp_port
        self.port = ""
        self.baud_rate = self._baud_rates[0] if self._baud_rates else 9600
        self._state = get_sub_state()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._serial = None
        self._ser_lock = threading.Lock()
        self._last_nmea_ts = 0.0

    @property
    def device_online(self) -> bool:
        with self._ser_lock:
            return self._serial is not None and self._serial.is_open

    def start(self) -> None:
        if not self._enabled:
            return
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True, name="gps-rx")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        self._close_serial()
        self._state.set_gps_scanning()

    def _close_serial(self) -> None:
        with self._ser_lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

    def _connect(self) -> bool:
        found = discover_gps_port(
            esp_port=self._esp_port,
            prefer_port=self._prefer_port,
            baud_rates=self._baud_rates,
        )
        if not found:
            self._state.set_gps_scanning()
            return False

        port, baud = found
        try:
            ser = open_serial_port(port, baud, timeout=0.2, write_timeout=0.2)
        except Exception as exc:
            print(f"[gps] Open failed ({port} @ {baud}): {exc}")
            self._state.set_gps_scanning()
            return False

        with self._ser_lock:
            self._serial = ser
        self.port = port
        self.baud_rate = baud
        self._last_nmea_ts = time.monotonic()
        self._state.set_gps_device(port)
        print(f"[gps] Auto-connected {port} @ {baud}")
        return True

    def _reader_loop(self) -> None:
        buf = b""
        self._state.set_gps_scanning()

        while not self._stop.is_set():
            if not self.device_online:
                buf = b""
                if not self._connect():
                    time.sleep(self._scan_interval)
                    continue

            stale = (
                self._last_nmea_ts > 0
                and time.monotonic() - self._last_nmea_ts > _DEVICE_STALE_S
            )
            if stale:
                print(f"[gps] No NMEA on {self.port} — rescanning")
                self._close_serial()
                self._state.set_gps_scanning()
                time.sleep(0.5)
                continue

            try:
                with self._ser_lock:
                    ser = self._serial
                    chunk = ser.read(256) if ser and ser.is_open else b""
            except Exception as exc:
                print(f"[gps] Read error on {self.port}: {exc}")
                self._close_serial()
                self._state.set_gps_scanning()
                time.sleep(self._scan_interval)
                continue

            if not chunk:
                time.sleep(0.05)
                continue

            self._last_nmea_ts = time.monotonic()
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", errors="ignore").strip("\r")
                if line.startswith("$"):
                    parse_nmea_line(line, self._state)


_reader: GpsReader | None = None
_reader_lock = threading.Lock()


def get_gps_reader(*, esp_port: str = "", autostart: bool = False) -> GpsReader:
    global _reader
    with _reader_lock:
        if _reader is None:
            _reader = GpsReader(esp_port=esp_port)
        elif esp_port:
            _reader._esp_port = esp_port
        if autostart:
            _reader.start()
        return _reader


def connect_gps(*, esp_port: str = "", autostart: bool = True) -> GpsReader:
    """
    Start background GPS auto-scan. Finds a USB GPS when plugged in — no manual port needed.
    """
    reader = get_gps_reader(esp_port=esp_port, autostart=autostart)
    if autostart and not (reader._thread and reader._thread.is_alive()):
        reader.start()
    return reader


def is_gps_enabled() -> bool:
    return _load_gps_config()["enabled"]


def latlon_to_local_m(
    lat: float,
    lon: float,
    origin_lat: float,
    origin_lon: float,
) -> tuple[float, float]:
    """Approximate local East/North metres from an origin fix."""
    lat_rad = math.radians(origin_lat)
    north_m = (lat - origin_lat) * 110_540.0
    east_m = (lon - origin_lon) * 111_320.0 * math.cos(lat_rad)
    return east_m, north_m


def _cli_main() -> int:
    print("[gps] Scanning for USB GPS modules …")
    cfg = _load_gps_config()
    candidates = list_gps_candidates(prefer_port=cfg["port"])
    if not candidates:
        print("[gps] No serial devices found. Plug in a USB GPS dongle and run again.")
        return 1

    print("[gps] Candidates:")
    for path in candidates[:8]:
        print(f"  {path}")

    found = discover_gps_port(
        prefer_port=cfg["port"],
        baud_rates=cfg["baud_rates"],
    )
    if not found:
        print("[gps] No NMEA source found on any candidate port.")
        print("     Ensure the GPS has sky view and is not sharing the ESP32 USB port.")
        return 1

    port, baud = found
    print(f"[gps] Found NMEA on {port} @ {baud}")
    reader = connect_gps(autostart=True)
    reader._prefer_port = port
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        snap = get_sub_state().telemetry_snapshot()["gps"]
        if snap.get("lat") is not None:
            print(
                f"[gps] Fix: {snap['lat']:.6f}, {snap['lon']:.6f} "
                f"sats={snap.get('satellites')} port={snap.get('port')}"
            )
            return 0
        time.sleep(0.5)

    print("[gps] Device online but no fix yet — move to open sky if possible.")
    return 0


if __name__ == "__main__":
    raise SystemExit(_cli_main())
