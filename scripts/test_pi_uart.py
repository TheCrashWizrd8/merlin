#!/usr/bin/env python3
"""
test_pi_uart.py
---------------
Listen on Pi GPIO UART (/dev/serial0) for ESP TEL lines or print raw bytes.

Usage:
  python scripts/test_pi_uart.py
  python scripts/test_pi_uart.py --port /dev/serial0 --seconds 10
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Test Pi GPIO UART to ESP")
    p.add_argument("--port", default="/dev/serial0", help="Serial device (default /dev/serial0)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--seconds", type=float, default=0, help="Stop after N seconds (0 = until Ctrl+C)")
    p.add_argument("--send", default=None, help="Send one line to ESP then listen (e.g. PING)")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        from src.esp_bridge import parse_telemetry_line
        from src.serial_util import is_gpio_uart_port, open_serial_port
        from src.sub_state import get_sub_state
    except ImportError as exc:
        print(f"[uart] {exc}", file=sys.stderr)
        return 1

    if not is_gpio_uart_port(args.port):
        print(f"[uart] Note: {args.port} is not the usual GPIO UART path (/dev/serial0)")

    try:
        ser = open_serial_port(args.port, args.baud, timeout=0.2, write_timeout=0.2)
    except Exception as exc:
        print(f"[uart] Cannot open {args.port}: {exc}", file=sys.stderr)
        print("Run: sudo bash scripts/setup_pi_uart.sh  then reboot")
        print("Ensure user is in dialout: groups")
        return 1

    state = get_sub_state()
    print(f"[uart] Listening on {args.port} @ {args.baud} (Ctrl+C to stop)")

    if args.send:
        line = args.send if args.send.endswith("\n") else args.send + "\n"
        ser.write(line.encode("ascii"))
        ser.flush()
        print(f"[uart] Sent: {args.send}")

    t0 = time.monotonic()
    try:
        while True:
            if args.seconds > 0 and time.monotonic() - t0 >= args.seconds:
                break
            raw = ser.readline()
            if not raw:
                continue
            text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if not text:
                continue
            ok = parse_telemetry_line(text, state)
            tag = "TEL" if ok else "RAW"
            print(f"[{tag}] {text}")
    except KeyboardInterrupt:
        print("\n[uart] Stopped.")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
