#!/usr/bin/env python3
"""
sub_server.py
---------------
Standalone sub diagnostics dashboard (no YOLO required).

Starts the web UI, ESP serial bridge, and Xbox controller reader.
Optional USB camera feed for /video_feed.

Usage:
  python sub_server.py
  python sub_server.py --no-camera
  python sub_server.py --port 8080 --serial-port /dev/ttyACM0

Open http://<pi-ip>:8080/sub/ in a browser.
"""

from __future__ import annotations

import argparse
import signal
import sys
import threading
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Sub diagnostics dashboard server")
    p.add_argument("--host", default="0.0.0.0", help="Bind address (default 0.0.0.0)")
    p.add_argument("--port", type=int, default=8080, help="HTTP port (default 8080)")
    p.add_argument("--serial-port", default=None, help="ESP32 serial port override")
    p.add_argument("--no-camera", action="store_true", help="Skip USB camera (placeholder stream)")
    p.add_argument("--camera-device", type=int, default=0, help="USB camera index")
    p.add_argument("--no-xbox", action="store_true", help="Skip Xbox controller polling")
    p.add_argument("--no-esp", action="store_true", help="Skip ESP serial bridge")
    return p.parse_args()


def _print_urls(host: str, port: int) -> None:
    print(f"[sub] Dashboard: http://localhost:{port}/sub/")
    if host == "0.0.0.0":
        try:
            import socket
            lan = socket.gethostbyname(socket.gethostname())
            if lan and lan != "127.0.0.1":
                print(f"[sub] Dashboard (LAN): http://{lan}:{port}/sub/")
        except Exception:
            pass
        try:
            import subprocess
            r = subprocess.run(
                ["tailscale", "ip", "-4"],
                capture_output=True, text=True, timeout=2,
            )
            if r.returncode == 0 and r.stdout.strip():
                print(f"[sub] Dashboard (Tailscale): http://{r.stdout.strip()}:{port}/sub/")
        except Exception:
            pass


def _camera_loop(device: int, running: threading.Event) -> None:
    from src.camera import Camera, CameraError
    from src.web_stream import set_latest_frame

    try:
        cam = Camera(device=device, width=640, height=480)
        cam.open()
    except CameraError as exc:
        print(f"[sub] Camera unavailable: {exc}")
        return

    print(f"[sub] Camera streaming from device {device}")
    try:
        while running.is_set():
            ok, frame = cam.read()
            if ok and frame is not None:
                set_latest_frame(frame)
            else:
                time.sleep(0.05)
    finally:
        cam.release()


def main() -> None:
    args = parse_args()

    try:
        from src.web_stream import _get_app, register_sub_dashboard, run_server
    except ImportError as e:
        if "flask" in str(e).lower():
            print("[ERROR] Flask required. pip install flask")
        else:
            print(f"[ERROR] {e}")
        sys.exit(1)

    _get_app()

    if not args.no_esp:
        from src.esp_bridge import get_esp_bridge
        bridge = get_esp_bridge(port=args.serial_port, autostart=False)
        bridge.start()
        print(f"[sub] ESP bridge on {bridge.port}")

    if not args.no_xbox:
        from src.xbox_controller import get_xbox_controller
        get_xbox_controller(autostart=False).start()
        print("[sub] Xbox controller polling started")

    register_sub_dashboard(start_services=False)

    running = threading.Event()
    running.set()

    if not args.no_camera:
        cam_thread = threading.Thread(
            target=_camera_loop,
            args=(args.camera_device, running),
            daemon=True,
        )
        cam_thread.start()

    def _shutdown(sig, frame):
        print("\n[sub] Shutting down …")
        running.clear()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    _print_urls(args.host, args.port)
    print("[sub] Press Ctrl+C to stop.")
    run_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
