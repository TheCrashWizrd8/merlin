#!/usr/bin/env python3
"""Probe Pi GPIO UART for ESP32 sub_rc responses (PING + passive listen)."""

from __future__ import annotations

import argparse
import sys
import time

try:
    import serial
except ImportError:
    print("Install pyserial: .venv/bin/pip install pyserial", file=sys.stderr)
    raise SystemExit(1)


def probe(port: str, listen_s: float = 3.0) -> bool:
    print(f"\n=== {port} ===")
    try:
        ser = serial.Serial(port, 115200, timeout=0.2)
    except Exception as exc:
        print(f"  open failed: {exc}")
        return False

    time.sleep(0.2)
    ser.reset_input_buffer()

    print(f"  listening {listen_s:.0f}s for spontaneous TEL lines...")
    t0 = time.time()
    spontaneous: list[str] = []
    while time.time() - t0 < listen_s:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            spontaneous.append(line)

    ser.write(b"PING\n")
    ser.flush()
    print("  sent PING")
    replies: list[str] = []
    t1 = time.time()
    while time.time() - t1 < 2.0:
        line = ser.readline().decode(errors="replace").strip()
        if line:
            replies.append(line)

    ser.close()

    if spontaneous:
        print("  spontaneous RX:")
        for line in spontaneous[:8]:
            print(f"    {line}")
    else:
        print("  no spontaneous telemetry")

    if replies:
        print("  after PING:")
        for line in replies[:8]:
            print(f"    {line}")
    else:
        print("  no PING reply")

    ok = any("OK PONG" in l for l in replies) or any(l.startswith("TEL ") for l in spontaneous)
    print("  RESULT:", "OK" if ok else "no ESP response")
    return ok


def main() -> int:
    p = argparse.ArgumentParser(description="Probe ESP over Pi GPIO UART")
    p.add_argument(
        "--port",
        action="append",
        help="Serial port (default: /dev/ttyAMA0 then /dev/ttyAMA10)",
    )
    args = p.parse_args()
    ports = args.port or ["/dev/ttyAMA0", "/dev/ttyAMA10"]
    any_ok = any(probe(port) for port in ports)
    if not any_ok:
        print(
            "\nNo response. Check:\n"
            "  1. ESP powered and running sub_rc.ino with USE_PI_UART=1\n"
            "  2. Pi pin 8 (TX) -> ESP RX (GPIO 44), pin 10 (RX) <- ESP TX (GPIO 43), GND shared\n"
            "  3. Pi 5: dtparam=uart0 in /boot/firmware/config.txt, then reboot\n"
            "  4. Flash firmware: bash esp32/upload_from_pi.sh scan"
        )
    return 0 if any_ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
