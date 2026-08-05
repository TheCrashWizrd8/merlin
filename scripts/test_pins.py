#!/usr/bin/env python3
"""
test_pins.py
------------
Run ESP pin-confirmation diagnostics over Pi UART (or USB serial).

Sends PING, PINS, and TEST commands from esp32/sub_rc/sub_rc.ino and reports
pass/fail. Use on the bench before wiring confirmation, or via GPIO UART on
the vehicle.

Usage:
  python scripts/test_pins.py
  python scripts/test_pins.py --port /dev/serial0
  python scripts/test_pins.py --port /dev/ttyACM0 --interactive
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

CHECKLIST: list[tuple[str, str, float]] = [
    ("PING", "OK PONG", 0.5),
    ("PINS", "OK PINS", 1.0),
    ("TEST S 0 0", "OK TEST S", 0.4),
    ("TEST S 1 0", "OK TEST S", 0.4),
    ("TEST S 2 0", "OK TEST S", 0.4),
    ("TEST S 3 0", "OK TEST S", 0.4),
    ("TEST T 0", "OK TEST T", 0.4),
    ("TEST B stop", "OK TEST B", 0.4),
    ("TEST L", "OK LEAK", 0.4),
    ("TEST A", "OK ADC", 0.4),
]

INTERACTIVE_COMMANDS = {
    "ping": "PING",
    "pins": "PINS",
    "help": "HELP",
    "s0": "TEST S 0 1.0",
    "s1": "TEST S 1 1.0",
    "s2": "TEST S 2 1.0",
    "s3": "TEST S 3 1.0",
    "thr-f": "TEST T 0.5",
    "thr-r": "TEST T -0.5",
    "thr-stop": "TEST T 0",
    "ballast-fill": "TEST B fill",
    "ballast-drain": "TEST B drain",
    "ballast-stop": "TEST B stop",
    "leaks": "TEST L",
    "adc": "TEST A",
}


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ESP sub pin confirmation diagnostics")
    p.add_argument("--port", default="/dev/serial0", help="Serial device")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--interactive", action="store_true", help="Interactive command shell")
    p.add_argument("--timeout", type=float, default=2.0, help="Per-command response timeout (s)")
    return p.parse_args()


def send_and_wait(ser, cmd: str, expect: str, timeout: float) -> tuple[bool, list[str]]:
    """Send one line and collect RX until expect seen or timeout."""
    if not cmd.endswith("\n"):
        cmd += "\n"
    ser.reset_input_buffer()
    ser.write(cmd.encode("ascii"))
    ser.flush()
    print(f"  TX: {cmd.rstrip()}")

    deadline = time.monotonic() + timeout
    lines: list[str] = []
    capturing_pins = False
    pins_done = False

    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        text = raw.decode("utf-8", errors="replace").rstrip("\r\n")
        if not text:
            continue
        lines.append(text)
        print(f"  RX: {text}")

        if text == "OK PINS BEGIN":
            capturing_pins = True
            continue
        if text == "OK PINS END":
            capturing_pins = False
            pins_done = True
            if expect == "OK PINS":
                return True, lines
            continue
        if capturing_pins:
            continue
        if expect in text:
            return True, lines

    if expect == "OK PINS" and pins_done:
        return True, lines
    return False, lines


def run_checklist(ser, timeout: float) -> int:
    passed = 0
    failed = 0
    print(f"\n=== Pin confirmation checklist ({len(CHECKLIST)} steps) ===\n")

    for cmd, expect, wait in CHECKLIST:
        print(f"[{cmd}] expecting {expect!r}")
        ok, _ = send_and_wait(ser, cmd, expect, wait + timeout)
        if ok:
            print("  PASS\n")
            passed += 1
        else:
            print("  FAIL\n")
            failed += 1
        time.sleep(0.15)

    print(f"=== Results: {passed} passed, {failed} failed ===")
    return 0 if failed == 0 else 1


def interactive_shell(ser) -> None:
    print("\nInteractive mode. Commands:", ", ".join(sorted(INTERACTIVE_COMMANDS)))
    print("Or type a raw ESP line. 'quit' to exit.\n")
    while True:
        try:
            user = input("pin> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not user:
            continue
        if user.lower() in ("q", "quit", "exit"):
            break
        cmd = INTERACTIVE_COMMANDS.get(user.lower(), user)
        ok, lines = send_and_wait(ser, cmd, "", 1.5)
        if not lines:
            print("  (no response)")


def main() -> int:
    args = parse_args()

    try:
        from src.serial_util import open_serial_port
    except ImportError as exc:
        print(f"[pins] {exc}", file=sys.stderr)
        return 1

    try:
        ser = open_serial_port(args.port, args.baud, timeout=0.2, write_timeout=0.5)
    except Exception as exc:
        print(f"[pins] Cannot open {args.port}: {exc}", file=sys.stderr)
        print("Run: sudo bash scripts/setup_pi_uart.sh  then reboot")
        return 1

    print(f"[pins] Connected to {args.port} @ {args.baud}")
    try:
        if args.interactive:
            interactive_shell(ser)
            return 0
        return run_checklist(ser, args.timeout)
    finally:
        ser.close()


if __name__ == "__main__":
    raise SystemExit(main())
