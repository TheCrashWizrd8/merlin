#!/usr/bin/env python3
"""
Quick L298N motor test over ESP32 serial protocol.

Protocol sent to ESP32:
  S <steer> D <drive> T <tilt>\n

This script keeps steer/tilt centered and only changes D by default.
"""

from __future__ import annotations

import argparse
import sys
import time


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Test motor via ESP32 serial protocol")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port (default: /dev/ttyACM0)")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")
    p.add_argument(
        "--mode",
        choices=["pulse", "sweep"],
        default="pulse",
        help="pulse: forward/reverse bursts, sweep: -max..+max ramp",
    )
    p.add_argument(
        "--speed",
        type=float,
        default=0.35,
        help="Drive magnitude 0.0..1.0 for pulse mode (default: 0.35)",
    )
    p.add_argument(
        "--max-speed",
        type=float,
        default=0.5,
        help="Max magnitude 0.0..1.0 for sweep mode (default: 0.5)",
    )
    p.add_argument(
        "--hold",
        type=float,
        default=1.2,
        help="Seconds to hold each pulse step (default: 1.2)",
    )
    p.add_argument(
        "--step",
        type=float,
        default=0.1,
        help="Sweep step size (default: 0.1)",
    )
    p.add_argument(
        "--pause",
        type=float,
        default=0.6,
        help="Pause between steps in seconds (default: 0.6)",
    )
    p.add_argument("--cycles", type=int, default=3, help="Number of cycles to run (default: 3)")
    p.add_argument("--steer", type=float, default=0.0, help="Steering value -1..1 (default: 0)")
    p.add_argument("--tilt", type=float, default=0.0, help="Tilt value -1..1 (default: 0)")
    p.add_argument(
        "--readback",
        action="store_true",
        help="Print response lines from ESP32 after each command",
    )
    return p


def clamp(v: float) -> float:
    return max(-1.0, min(1.0, v))


def send_cmd(ser, steer: float, drive: float, tilt: float, readback: bool) -> None:
    line = f"S {clamp(steer):.3f} D {clamp(drive):.3f} T {clamp(tilt):.3f}\n"
    ser.write(line.encode("ascii"))
    ser.flush()
    print(line, end="")
    if readback:
        time.sleep(0.03)
        reply = ser.readline().decode("utf-8", errors="replace").strip()
        if reply:
            print(f"  <- {reply}")


def run_pulse(args, ser) -> None:
    speed = abs(clamp(args.speed))
    for i in range(args.cycles):
        print(f"\nCycle {i + 1}/{args.cycles}")
        send_cmd(ser, args.steer, +speed, args.tilt, args.readback)
        time.sleep(args.hold)
        send_cmd(ser, args.steer, 0.0, args.tilt, args.readback)
        time.sleep(args.pause)
        send_cmd(ser, args.steer, -speed, args.tilt, args.readback)
        time.sleep(args.hold)
        send_cmd(ser, args.steer, 0.0, args.tilt, args.readback)
        time.sleep(args.pause)


def frange(start: float, stop: float, step: float):
    x = start
    if step == 0:
        raise ValueError("step must not be zero")
    if step > 0:
        while x <= stop + 1e-9:
            yield x
            x += step
    else:
        while x >= stop - 1e-9:
            yield x
            x += step


def run_sweep(args, ser) -> None:
    max_speed = abs(clamp(args.max_speed))
    step = abs(args.step)
    for i in range(args.cycles):
        print(f"\nCycle {i + 1}/{args.cycles}")
        for d in frange(-max_speed, max_speed, step):
            send_cmd(ser, args.steer, d, args.tilt, args.readback)
            time.sleep(args.pause)
        for d in frange(max_speed, -max_speed, -step):
            send_cmd(ser, args.steer, d, args.tilt, args.readback)
            time.sleep(args.pause)
        send_cmd(ser, args.steer, 0.0, args.tilt, args.readback)
        time.sleep(args.pause)


def main() -> int:
    args = build_parser().parse_args()

    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2, write_timeout=0.2)
    except Exception as e:
        print(f"Could not open serial port {args.port}: {e}", file=sys.stderr)
        return 1

    print(f"Opened {args.port} @ {args.baud}")
    print("Testing motor. Press Ctrl+C to stop.")

    try:
        # Let ESP32 reset/open its USB serial cleanly.
        time.sleep(1.5)
        send_cmd(ser, args.steer, 0.0, args.tilt, args.readback)
        time.sleep(0.2)
        if args.mode == "pulse":
            run_pulse(args, ser)
        else:
            run_sweep(args, ser)
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        try:
            send_cmd(ser, args.steer, 0.0, args.tilt, args.readback)
            time.sleep(0.1)
        except Exception:
            pass
        ser.close()
        print("Motor stopped, serial closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
