#!/usr/bin/env python3
"""
test_telemetry.py
-----------------
SIMULATION ONLY — fake sensor data for UI development without ESP hardware.

For real hardware use:
  python sub_server.py

Usage:
  python scripts/test_telemetry.py
  python scripts/test_telemetry.py --port 8080
  python scripts/test_telemetry.py --leak-alarm   # flash leak warning every 20s

Open http://localhost:8080/sub/
"""

from __future__ import annotations

import argparse
import math
import signal
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sub dashboard with simulated telemetry")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--hz", type=float, default=5.0, help="Telemetry update rate")
    p.add_argument("--no-camera", action="store_true", help="Skip USB camera feed")
    p.add_argument("--camera-device", type=int, default=0, help="USB camera index")
    p.add_argument(
        "--leak-alarm",
        action="store_true",
        help="Toggle leak sensor 3 every 20s to test the alarm flash",
    )
    return p.parse_args()


def _camera_loop(device: int, running: threading.Event) -> None:
    from src.camera import Camera, CameraError
    from src.web_stream import set_latest_frame

    try:
        cam = Camera(device=device, width=640, height=480)
        cam.open()
    except CameraError as exc:
        print(f"[test] Camera unavailable: {exc}")
        return

    print(f"[test] Camera streaming from device {device}")
    try:
        while running.is_set():
            ok, frame = cam.read()
            if ok and frame is not None:
                set_latest_frame(frame)
            else:
                time.sleep(0.05)
    finally:
        cam.release()


def telemetry_loop(state, hz: float, leak_alarm: bool, stop: threading.Event) -> None:
    from src.sub_state import SubActuators, XboxState

    t0 = time.monotonic()
    ballast = 0.5

    state.set_esp_connected(True, "test://simulated")
    state.append_serial("sys", "Test telemetry generator started")

    while not stop.is_set():
        t = time.monotonic() - t0

        battery = 12.6 - (t % 180.0) * 0.004
        depth = 1.5 + 0.5 * math.sin(t * 0.15)
        pitch = 8.0 * math.sin(t * 0.4)
        roll = 5.0 * math.sin(t * 0.25)
        yaw = (t * 12.0) % 360.0

        leak3 = False
        if leak_alarm:
            leak3 = int(t // 20) % 2 == 1

        ballast = max(0.0, min(1.0, ballast + 0.02 * math.sin(t * 0.1)))

        state.update_battery(battery)
        state.update_gyro(pitch, roll, yaw)
        state.update_depth(depth)
        state.update_leaks([False, False, leak3, False])
        state.update_ballast_level(ballast, tank="fore")
        state.update_ballast_level(ballast * 0.9, tank="aft")

        # Fake serial log lines (same format as real ESP)
        state.append_serial("tx", f"TEL battery {battery:.2f}")
        state.append_serial("tx", f"TEL gyro {pitch:.2f} {roll:.2f} {yaw:.2f}")
        state.append_serial("tx", f"TEL depth {depth:.2f}")
        state.append_serial("tx", f"TEL leak 0 0 {1 if leak3 else 0} 0")
        state.append_serial("tx", f"TEL ballast {ballast:.3f}")

        # Fake Xbox + actuators so control panel has something to show
        stick_x = 0.6 * math.sin(t * 0.5)
        stick_y = 0.4 * math.cos(t * 0.35)
        state.update_xbox(XboxState(
            connected=True,
            name="Test Gamepad",
            left_stick_x=stick_x,
            left_stick_y=stick_y,
            right_stick_x=0.3 * math.sin(t * 0.2),
            right_stick_y=0.5 * math.cos(t * 0.3),
            triggers={"lt": 0.0, "rt": 0.0},
            last_update=time.time(),
        ))
        state.set_xbox_actuators(SubActuators(
            aft_steer_y=stick_x,
            aft_steer_z=-stick_y,
            thruster_x=0.5 * math.cos(t * 0.3),
            fin_left=0.2 * math.sin(t * 0.6),
            fin_right=-0.2 * math.sin(t * 0.6),
        ))
        state.set_control_mode("xbox")
        state.recompute_effective()

        time.sleep(1.0 / hz)


def main() -> int:
    args = parse_args()

    try:
        from src.web_stream import _get_app, register_sub_dashboard, run_server
        from src.sub_state import get_sub_state
    except ImportError as exc:
        print(f"[test] {exc}", file=sys.stderr)
        return 1

    _get_app()
    register_sub_dashboard(start_services=False)

    running = threading.Event()
    running.set()
    if not args.no_camera:
        threading.Thread(
            target=_camera_loop,
            args=(args.camera_device, running),
            daemon=True,
            name="test-camera",
        ).start()

    stop = threading.Event()
    state = get_sub_state()
    thread = threading.Thread(
        target=telemetry_loop,
        args=(state, args.hz, args.leak_alarm, stop),
        daemon=True,
        name="test-telemetry",
    )
    thread.start()

    def _shutdown(sig, frame):
        print("\n[test] Stopping.")
        running.clear()
        stop.set()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[test] Simulated telemetry @ {args.hz} Hz")
    if args.leak_alarm:
        print("[test] Leak alarm demo: sensor L3 toggles every 20s")
    print(f"[test] Dashboard: http://localhost:{args.port}/sub/")
    print("[test] Ctrl+C to stop.")

    run_server(host=args.host, port=args.port)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
