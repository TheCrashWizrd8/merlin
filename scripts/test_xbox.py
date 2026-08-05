#!/usr/bin/env python3
"""
test_xbox.py
------------
Check that a USB or Bluetooth Xbox controller is visible to SDL/pygame.

Usage:
  python scripts/test_xbox.py
  python scripts/test_xbox.py --watch   # live stick values until Ctrl+C
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Test Xbox / gamepad detection")
    p.add_argument("--watch", action="store_true", help="Print live stick values")
    p.add_argument("--interval", type=float, default=0.1, help="Watch refresh (s)")
    return p.parse_args()


def list_devices() -> None:
    os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

    import pygame
    from pygame._sdl2 import controller as gc

    pygame.init()
    pygame.joystick.init()
    gc.init()

    js_count = pygame.joystick.get_count()
    gc_count = gc.get_count()
    print(f"Joysticks: {js_count}  |  GameControllers: {gc_count}")

    if js_count == 0:
        print("\nNo gamepad found.")
        print("Bluetooth Xbox on Pi usually needs:")
        print("  1. Pair in bluetoothctl (hold controller sync button)")
        print("  2. xpadneo driver for Xbox One / Series BT")
        print("  3. User in 'input' group: sudo usermod -aG input $USER")
        print("See docs/SUB_DASHBOARD.md#xbox-controller-bluetooth")
        return

    for i in range(js_count):
        name = pygame.joystick.Joystick(i).get_name()
        gc_ok = gc.is_gamecontroller(i)
        print(f"  [{i}] {name!r}  gamecontroller={gc_ok}")

    idx = 0
    if gc.is_gamecontroller(idx):
        pad = gc.Controller(idx)
        print(f"\nUsing GameController API: {pad.name}")
    else:
        joy = pygame.joystick.Joystick(idx)
        joy.init()
        print(f"\nUsing Joystick fallback: {joy.get_name()} ({joy.get_numaxes()} axes)")


def watch_loop(interval: float) -> None:
    from src.xbox_controller import _GamepadBackend, map_xbox_to_actuators

    pad = _GamepadBackend(device_index=None)
    if not pad.open():
        print("[xbox] No controller — pair Bluetooth or plug in USB, then retry.")
        return 1

    print(f"[xbox] Watching {pad.name} (Ctrl+C to stop)\n")
    try:
        while True:
            x = pad.read()
            act = map_xbox_to_actuators(x)
            print(
                f"\r LS {x.left_stick_x:+.2f} {x.left_stick_y:+.2f}"
                f"  RS {x.right_stick_x:+.2f} {x.right_stick_y:+.2f}"
                f"  LT {x.triggers.get('lt', 0):.2f} RT {x.triggers.get('rt', 0):.2f}"
                f"  thr {act.thruster_x:+.2f} steer {act.aft_steer_y:+.2f}",
                end="",
                flush=True,
            )
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\n[xbox] Done.")
    finally:
        pad.close()
    return 0


def main() -> int:
    args = parse_args()
    if args.watch:
        return watch_loop(args.interval)
    list_devices()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
