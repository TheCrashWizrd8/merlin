#!/usr/bin/env python3
"""
Interactive serial terminal for ESP32 testing.

Examples:
  python3 esp32/serial_talk.py --port /dev/ttyACM0
  python3 esp32/serial_talk.py --scan
"""

from __future__ import annotations

import argparse
import glob
import sys
import threading
import time


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Open serial port and chat with ESP32")
    p.add_argument("--port", default="/dev/ttyACM0", help="Serial port path")
    p.add_argument("--baud", type=int, default=115200, help="Baud rate")
    p.add_argument("--scan", action="store_true", help="List likely serial ports and exit")
    p.add_argument(
        "--no-reset-wait",
        action="store_true",
        help="Skip initial wait after opening port",
    )
    return p


def scan_ports() -> list[str]:
    ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    return ports


def reader_loop(ser, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = ser.readline()
        except Exception:
            break
        if not line:
            continue
        text = line.decode("utf-8", errors="replace").rstrip("\r\n")
        print(f"< {text}")


def main() -> int:
    args = build_parser().parse_args()

    if args.scan:
        ports = scan_ports()
        if not ports:
            print("No /dev/ttyACM* or /dev/ttyUSB* ports found.")
            return 1
        print("Detected serial ports:")
        for p in ports:
            print(f"  {p}")
        return 0

    try:
        import serial
    except ImportError:
        print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        return 1

    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=0.1,
            write_timeout=2.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
    except Exception as e:
        print(f"Could not open {args.port}: {e}", file=sys.stderr)
        return 1

    # Avoid accidental reset/flow-control gating on some USB CDC setups.
    try:
        ser.setDTR(False)
        ser.setRTS(False)
    except Exception:
        pass

    print(f"Opened {args.port} @ {args.baud}")
    print("Type text and press Enter to send.")
    print("Commands: /quit to exit, /help for tips")

    if not args.no_reset_wait:
        # Many ESP32 boards reset when serial opens.
        time.sleep(1.5)

    stop_event = threading.Event()
    thread = threading.Thread(target=reader_loop, args=(ser, stop_event), daemon=True)
    thread.start()

    try:
        while True:
            try:
                user = input("> ")
            except EOFError:
                break

            cmd = user.strip()
            if cmd == "/quit":
                break
            if cmd == "/help":
                print("Send raw lines, for example:")
                print("  S 0.000 D 0.300 T 0.000")
                print("  1   (for apple_car_hw_test forward pulse)")
                print("  2   (reverse pulse), 0 (stop), s (servo sweep), m (motor cycle)")
                continue
            if not user:
                continue

            try:
                ser.write((user + "\n").encode("utf-8"))
                ser.flush()
            except serial.SerialTimeoutException:
                # Some boards briefly stall after open/reset; recover once.
                print("Write timeout, retrying once...")
                try:
                    time.sleep(0.4)
                    ser.reset_output_buffer()
                    ser.write((user + "\n").encode("utf-8"))
                    ser.flush()
                except Exception as e:
                    print(f"Write failed after retry: {e}", file=sys.stderr)
                    break
            except Exception as e:
                print(f"Write failed: {e}", file=sys.stderr)
                break
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        try:
            ser.close()
        except Exception:
            pass
        print("\nSerial closed.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
