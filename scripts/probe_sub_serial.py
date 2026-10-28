#!/usr/bin/env python3
"""
Find which serial port talks to ESP32 sub_rc (PING / TEL).

Run on the Pi with the ESP powered and linked the way you use in the vehicle:

  USB:  ESP USB cable → Pi USB port, firmware USE_PI_UART=0 (default)
  UART: Pi GPIO 8/10 ↔ ESP 44/43, firmware USE_PI_UART=1

  cd ~/yolo-project && .venv/bin/python scripts/probe_sub_serial.py
"""

from __future__ import annotations

import glob
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.serial_util import (  # noqa: E402
    is_esp_usb_device,
    is_gps_usb_device,
    is_gpio_uart_port,
    open_serial_port,
    serial_by_id_path,
)


def _candidate_ports() -> list[str]:
    seen: set[str] = set()
    out: list[str] = []

    def add(p: str) -> None:
        if not p or p in seen:
            return
        try:
            if is_gpio_uart_port(p):
                if Path(p).exists():
                    seen.add(p)
                    out.append(p)
                return
            if not Path(p).exists():
                return
        except OSError:
            return
        seen.add(p)
        out.append(p)

    for p in sorted(glob.glob("/dev/serial/by-id/*")):
        if is_gps_usb_device(p):
            continue
        add(p)
    for p in sorted(glob.glob("/dev/ttyACM*")) + sorted(glob.glob("/dev/ttyUSB*")):
        if is_gps_usb_device(p):
            continue
        add(p)
    for p in ("/dev/serial0", "/dev/ttyAMA0", "/dev/ttyAMA10"):
        add(p)
    return out


def probe(port: str, listen_s: float = 2.5) -> bool:
    label = port
    if port.startswith("/dev/tty"):
        bid = serial_by_id_path(port)
        if bid != port:
            label = f"{port} ({bid})"
    print(f"\n=== {label} ===")
    try:
        ser = open_serial_port(port, 115200, timeout=0.25, write_timeout=1.0)
    except Exception as exc:
        print(f"  open failed: {exc}")
        return False

    time.sleep(0.35)
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    spontaneous: list[str] = []
    t0 = time.time()
    while time.time() - t0 < listen_s:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line:
            spontaneous.append(line)

    ser.write(b"PING\n")
    ser.flush()
    replies: list[str] = []
    t1 = time.time()
    while time.time() - t1 < 2.0:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode(errors="replace").strip()
        if line:
            replies.append(line)

    ser.close()

    if spontaneous:
        print("  spontaneous RX (first lines):")
        for line in spontaneous[:6]:
            print(f"    {line}")
    else:
        print("  no spontaneous lines (no TEL / boot text)")

    if replies:
        print("  after PING:")
        for line in replies[:6]:
            print(f"    {line}")
    else:
        print("  no PING reply")

    ok = any("OK PONG" in l for l in replies) or any(
        l.startswith("TEL ") or l.startswith("sub_rc") for l in spontaneous + replies
    )
    espish = is_esp_usb_device(port) or is_gpio_uart_port(port)
    print("  RESULT:", "ESP OK" if ok else ("wrong device?" if not ok and not espish else "no response"))
    if ok:
        print(f"\n  → Put in config/hardware.yaml under sub_serial.port:")
        print(f'      port: "{serial_by_id_path(port) if port.startswith("/dev/tty") else port}"')
    return ok


def main() -> int:
    ports = _candidate_ports()
    if not ports:
        print("No serial ports found.")
        print("  USB: plug ESP into the Pi (not only your laptop) and check ls /dev/ttyACM*")
        print("  UART: enable Pi UART (/dev/serial0) and flash sub_rc with USE_PI_UART=1")
        return 1

    print("Probing serial ports for ESP32 sub_rc …")
    winners = [p for p in ports if probe(p)]
    if winners:
        print(f"\nSuccess on: {winners[0]}")
        return 0

    print(
        "\nNo ESP response on any port.\n"
        "Checklist:\n"
        "  1. Same cable that works on your PC — data wires, not charge-only.\n"
        "  2. USB to the **Pi** for runtime (laptop USB does not connect the sub stack).\n"
        "  3. Firmware link mode:\n"
        "       USB (default): sub_rc.ino USE_PI_UART=0, ESP USB → Pi.\n"
        "       GPIO UART: USE_PI_UART=1, Pi pin 8→ESP RX(44), pin 10←ESP TX(43), common GND.\n"
        "  4. ls -la /dev/serial/by-id/ — expect Espressif / USB JTAG.\n"
        "  5. Reflash from Pi: bash esp32/upload_from_pi.sh scan\n"
    )
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
