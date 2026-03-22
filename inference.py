"""
inference.py
------------
Main inference loop.  Run this on the Raspberry Pi 5.

What it does each frame
-----------------------
1.  Capture frame from USB camera  (src/camera.py)
2.  Run YOLO detection             (src/detector.py)
3.  Pick best apple + compute error (src/tracker.py)
4.  Compute servo/motor commands   (src/controller.py)
5.  Print ControlOutput table      (controller.ControlOutput.pretty())
6.  Draw annotated camera view     (src/display.py)

Usage
-----
    # With display window (requires X forwarding or local monitor)
    python inference.py

    # Headless (SSH without X forwarding — prints to terminal only)
    python inference.py --headless

    # Different camera device
    python inference.py --device 1

    # Skip printing every frame (only print when apple is detected)
    python inference.py --print-on-detect

    # Serve live stream in browser (no OpenCV window)
    python inference.py --web
    # Then open http://<pi-ip>:5000

Press q in the camera window to quit (or Ctrl-C in headless mode).
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
    parser = argparse.ArgumentParser(
        description="YOLO apple detection inference on Raspberry Pi 5."
    )
    parser.add_argument(
        "--device", type=int, default=0,
        help="USB camera device index (default: 0)",
    )
    parser.add_argument(
        "--width", type=int, default=640,
        help="Camera capture width (default: 640)",
    )
    parser.add_argument(
        "--height", type=int, default=480,
        help="Camera capture height (default: 480)",
    )
    parser.add_argument(
        "--headless", action="store_true",
        help="Run without an OpenCV display window (SSH / no monitor)",
    )
    parser.add_argument(
        "--print-on-detect", action="store_true",
        help="Only print ControlOutput when an apple is detected",
    )
    parser.add_argument(
        "--tracker-strategy",
        choices=["best_confidence", "closest_to_centre"],
        default="best_confidence",
        help="Which apple to track when multiple detections exist",
    )
    parser.add_argument(
        "--apple-label", type=str, default="apple",
        help="Class label string used by the dataset (default: 'apple')",
    )
    parser.add_argument(
        "--web", action="store_true",
        help="Serve live stream on a web page (no OpenCV window). Open http://<pi-ip>:5000",
    )
    parser.add_argument(
        "--web-host", type=str, default="0.0.0.0",
        help="Bind address for web server (default: 0.0.0.0 = all interfaces)",
    )
    parser.add_argument(
        "--web-port", type=int, default=8080,
        help="Port for web server (default: 8080; use if 5000 is in use)",
    )
    parser.add_argument(
        "--serial-verbose", action="store_true", default=True,
        help="Echo each S D T line sent to hardware (serial) to the console (default: True)",
    )
    parser.add_argument(
        "--no-serial-verbose", action="store_false", dest="serial_verbose",
        help="Disable echoing S D T lines to the console",
    )
    parser.add_argument(
        "--serial-port", type=str, default=None,
        help="Override serial port from config/hardware.yaml (e.g. /dev/ttyACM1)",
    )
    parser.add_argument(
        "--control-profile",
        choices=["config", "stable", "aggressive"],
        default="config",
        help="Control tuning profile override (default: config values from hardware.yaml)",
    )
    return parser.parse_args()


def _print_web_urls(web_host: str, web_port: int) -> None:
    """Print URLs for the web stream, including Tailscale IP when available."""
    if web_host != "0.0.0.0":
        print(f"[inference] Web stream: http://{web_host}:{web_port}")
        return
    print(f"[inference] Web stream (local): http://localhost:{web_port}")
    try:
        import socket
        lan = socket.gethostbyname(socket.gethostname())
        if lan and lan != "127.0.0.1":
            print(f"[inference] Web stream (LAN):  http://{lan}:{web_port}")
    except Exception:
        pass
    # Tailscale: so you can open from another device on your tailnet
    try:
        import subprocess
        r = subprocess.run(
            ["tailscale", "ip", "-4"],
            capture_output=True,
            text=True,
            timeout=2,
        )
        if r.returncode == 0 and r.stdout and r.stdout.strip():
            ts_ip = r.stdout.strip()
            print(f"[inference] Web stream (Tailscale): http://{ts_ip}:{web_port}")
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        pass


def main() -> None:
    args = parse_args()

    # ------------------------------------------------------------------
    # Import modules (deferred so path is set up first)
    # ------------------------------------------------------------------
    from src.camera import Camera, CameraError
    from src.detector import Detector
    from src.tracker import Tracker
    from src.controller import Controller
    from src.display import Display

    # ------------------------------------------------------------------
    # Initialise components
    # ------------------------------------------------------------------
    print("[inference] Initialising detector …")
    try:
        detector = Detector()
    except Exception as exc:
        print(f"[ERROR] Could not load model: {exc}")
        sys.exit(1)

    tracker    = Tracker(strategy=args.tracker_strategy)
    controller = Controller.from_hardware_config(profile=args.control_profile)
    if args.control_profile != "config":
        print(f"[inference] Control profile override: {args.control_profile}")
    # With --web we serve in browser so no OpenCV window
    display    = Display(headless=args.headless or args.web)

    from src.hardware import from_config as hardware_from_config
    try:
        hardware = hardware_from_config(serial_port_override=args.serial_port)
        print(f"[inference] Hardware: {type(hardware).__name__} (steering, drive, camera tilt)")
        if args.serial_port:
            print(f"[inference] Serial port override: {args.serial_port}")
    except Exception as exc:
        print(f"[ERROR] Hardware init failed: {exc}")
        sys.exit(1)

    if args.web:
        try:
            from src.web_stream import _get_app, set_latest_frame, run_server
        except ImportError as e:
            if "flask" in str(e).lower():
                print("[ERROR] Flask is required for --web. Install with: pip install flask")
            else:
                print(f"[ERROR] {e}")
            sys.exit(1)
        _get_app()  # ensure app is created
        server_thread = threading.Thread(
            target=run_server,
            kwargs={"host": args.web_host, "port": args.web_port},
            daemon=True,
        )
        server_thread.start()
        time.sleep(0.5)  # give server time to bind
        _print_web_urls(args.web_host, args.web_port)

    print(f"[inference] Opening camera device {args.device} …")
    cam = Camera(device=args.device, width=args.width, height=args.height)
    try:
        cam.open()
    except CameraError as exc:
        print(f"[ERROR] {exc}")
        sys.exit(1)

    # ------------------------------------------------------------------
    # Graceful shutdown on Ctrl-C
    # ------------------------------------------------------------------
    running = True

    def _handle_signal(sig, frame):
        nonlocal running
        print("\n[inference] Shutting down …")
        running = False

    signal.signal(signal.SIGINT, _handle_signal)
    signal.signal(signal.SIGTERM, _handle_signal)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    print("\n[inference] Running.  Press q to quit.\n")
    frame_count = 0
    loop_start  = time.monotonic()

    try:
        while running:
            ok, frame = cam.read()
            if not ok or frame is None:
                print("[WARN] Failed to read frame — retrying …")
                time.sleep(0.05)
                continue

            frame_count += 1

            # 1. Detect
            detections = detector.detect(frame)

            # 2. Track
            track = tracker.update(
                detections,
                frame.shape,
                apple_label=args.apple_label,
            )

            # 3. Control
            output = controller.compute(track)
            if args.web:
                from dataclasses import replace
                from src.control_source import get_current_sdt, SDT
                sdt = get_current_sdt(SDT(output.steering_servo, output.drive_motor, output.camera_tilt_servo))
                output = replace(output, steering_servo=sdt.s, drive_motor=sdt.d, camera_tilt_servo=sdt.t)

            # 3b. Send to hardware (steering servo, drive motor, camera tilt servo)
            hardware.apply(output)
            serial_line = (
                f"S {output.steering_servo:.3f} "
                f"D {output.drive_motor:.3f} "
                f"T {output.camera_tilt_servo:.3f}\n"
            )

            # 4. Print
            if not args.print_on_detect or output.apple_detected:
                # Clear previous block only (table = 13 lines; +1 for serial when verbose)
                if frame_count > 1:
                    n_up = 14 if args.serial_verbose else 13
                    sys.stdout.write(f"\033[{n_up}A\033[J")
                print(output.pretty())
                if args.serial_verbose:
                    print(serial_line, end="", flush=True)
                sys.stdout.flush()

            # 5. Display
            annotated = display.draw(frame, output)
            if args.web:
                set_latest_frame(annotated)
            if not display.show(annotated):
                break   # user pressed q

    finally:
        cam.release()
        display.close()
        hardware.close()

        elapsed = time.monotonic() - loop_start
        avg_fps = frame_count / elapsed if elapsed > 0 else 0
        print(f"\n[inference] Processed {frame_count} frames in "
              f"{elapsed:.1f}s  ({avg_fps:.1f} fps avg)")


if __name__ == "__main__":
    main()
