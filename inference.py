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

    # Serve live stream in browser (no OpenCV window). With interface: sub in
    # config/hardware.yaml, also starts ESP telemetry bridge and /sub/ dashboard.
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
        help="Serve live stream on a web page (no OpenCV window). Default port 8080 (see --web-port)",
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
        "--serial-port", type=str, default=None,
        help="Override ESP32 serial port from config/hardware.yaml (e.g. /dev/ttyACM1)",
    )
    parser.add_argument(
        "--control-profile",
        choices=["config", "stable", "aggressive"],
        default="config",
        help="Control tuning profile override (default: config values from hardware.yaml)",
    )
    parser.add_argument(
        "--tolerate-missing-devices",
        action="store_true",
        help=(
            "Keep running when the USB camera or ESP32 serial device can't be opened. "
            "In this mode, YOLO frame processing is skipped if the camera is missing, "
            "and control falls back to manual s/d/t."
        ),
    )
    parser.add_argument(
        "--sub",
        action="store_true",
        help="Enable sub vehicle dashboard (/sub/), ESP bridge, and Xbox control",
    )
    parser.add_argument(
        "--no-sub",
        action="store_true",
        help="Disable sub stack even when config/hardware.yaml has interface: sub",
    )
    parser.add_argument(
        "--no-xbox",
        action="store_true",
        help="With --sub, skip Xbox controller polling",
    )
    parser.add_argument(
        "--timing",
        action="store_true",
        help="Log capture/YOLO/draw ms and loop FPS every 30 frames",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Skip per-frame ControlOutput table (default when /sub/ or --web is active)",
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


def _load_hw_config() -> dict:
    import yaml
    path = PROJECT_ROOT / "config" / "hardware.yaml"
    try:
        with open(path) as f:
            return yaml.safe_load(f) or {}
    except OSError:
        return {}


def _sub_stack_enabled(args: argparse.Namespace, hw_config: dict) -> bool:
    if args.no_sub:
        return False
    if args.sub:
        return True
    return (hw_config.get("interface") or "").strip().lower() == "sub"


def _start_sub_stack(args: argparse.Namespace, serial_port: str | None) -> object | None:
    """Start ESP bridge, optional Xbox polling, and /sub/ routes. Returns bridge or None."""
    from src.esp_bridge import get_esp_bridge
    from src.sub_state import get_sub_state

    bridge = get_esp_bridge(port=serial_port, autostart=False)
    bridge.start()
    print(f"[inference] ESP bridge on {bridge.port} (telemetry + S2/B actuators)")

    if not args.no_xbox:
        try:
            from src.xbox_controller import connect_xbox, is_xbox_enabled
            if is_xbox_enabled():
                connect_xbox()
                print("[inference] Xbox controller polling started (plug in pad anytime)")
        except ImportError:
            print("[WARN] Xbox controller unavailable — install pygame")

    # YOLO inference should drive actuators unless the dashboard overrides mode.
    get_sub_state().set_control_mode("auto")
    return bridge


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

    from src.hardware import from_config as hardware_from_config
    from src.hardware import StubOutput

    hw_config = _load_hw_config()
    use_sub = _sub_stack_enabled(args, hw_config)
    serve_http = args.web or use_sub
    # Browser/sub dashboard mode: skip OpenCV window (imshow over VNC/SSH kills FPS).
    display = Display(headless=args.headless or serve_http)
    quiet_terminal = args.quiet or serve_http
    esp_bridge = None

    hardware: object
    hardware_init_error: Exception | None = None
    try:
        hardware = hardware_from_config()
        label = type(hardware).__name__
        if use_sub:
            print(f"[inference] Hardware: {label} → sub_state (ESP bridge owns serial)")
        else:
            print(f"[inference] Hardware: {label}")
    except Exception as exc:
        if args.tolerate_missing_devices:
            hardware_init_error = exc
            hardware = StubOutput({})
            print(f"[WARN] Hardware init failed; falling back to stub: {exc}")
        else:
            print(f"[ERROR] Hardware init failed: {exc}")
            sys.exit(1)

    if serve_http:
        try:
            from src.web_stream import _get_app, set_latest_frame, run_server, register_sub_dashboard
        except ImportError as e:
            if "flask" in str(e).lower():
                print("[ERROR] Flask is required for --web/--sub. Install with: pip install flask")
            else:
                print(f"[ERROR] {e}")
            sys.exit(1)
        _get_app()
        if use_sub:
            register_sub_dashboard(start_services=False)
            esp_bridge = _start_sub_stack(args, args.serial_port)
            print(f"[inference] Sub dashboard: http://localhost:{args.web_port}/sub/")
        if args.web:
            server_thread = threading.Thread(
                target=run_server,
                kwargs={"host": args.web_host, "port": args.web_port},
                daemon=True,
            )
            server_thread.start()
            time.sleep(0.5)
            _print_web_urls(args.web_host, args.web_port)
        elif use_sub:
            server_thread = threading.Thread(
                target=run_server,
                kwargs={"host": args.web_host, "port": args.web_port},
                daemon=True,
            )
            server_thread.start()
            time.sleep(0.5)
            print(f"[inference] Sub server: http://{args.web_host}:{args.web_port}/sub/")
    elif use_sub:
        # Headless YOLO + ESP telemetry/actuators (no browser UI).
        esp_bridge = _start_sub_stack(args, args.serial_port)

    cam = None
    manual_only = False
    try:
        print(f"[inference] Opening camera device {args.device} …")
        cam = Camera(device=args.device, width=args.width, height=args.height)
        cam.open()
    except CameraError as exc:
        if args.tolerate_missing_devices:
            manual_only = True
            print(f"[WARN] Camera init failed; skipping YOLO loop (manual-only): {exc}")
        else:
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
    loop_start = time.monotonic()
    manual_loop_start = loop_start

    # control_source is the single source of truth for manual s/d/t.
    # In tolerate-missing mode, we keep outputs stuck in manual.
    from src.control_source import SDT, get_manual, set_mode
    last_hw_retry = 0.0
    hw_retry_interval_s = 5.0

    try:
        if manual_only:
            # "No camera" mode: keep the server alive and keep applying manual
            # s/d/t to hardware (or stub) without running YOLO/tracking.
            set_mode("manual")
            from src.controller import ControlOutput

            while running:
                # Try to re-init real hardware if we fell back to StubOutput.
                # This is additive and only runs in tolerate mode.
                if args.tolerate_missing_devices and hardware_init_error is not None:
                    now = time.monotonic()
                    if now - last_hw_retry >= hw_retry_interval_s:
                        last_hw_retry = now
                        try:
                            hardware = hardware_from_config()
                            hardware_init_error = None
                            print(f"[inference] Hardware reconnected: {type(hardware).__name__}")
                        except Exception as exc:
                            print(f"[inference] Still waiting for hardware: {exc}")

                sdt = get_manual()
                output = ControlOutput(
                    steering_servo=sdt.s,
                    drive_motor=sdt.d,
                    camera_tilt_servo=sdt.t,
                    apple_detected=False,
                    target_x=0,
                    target_y=0,
                    error_x=0.0,
                    error_y=0.0,
                    confidence=0.0,
                    timestamp=time.time(),
                )
                try:
                    hardware.apply(output)
                except Exception as exc:
                    if args.tolerate_missing_devices:
                        hardware_init_error = exc
                        hardware = StubOutput({})
                        # Keep outputs stuck in manual while HW is missing.
                        set_mode("manual")
                        print(f"[WARN] Hardware apply failed; using stub: {exc}")
                    else:
                        raise
                time.sleep(0.1)
        else:
            while running:
                # If hardware was unavailable at startup, retry periodically.
                if args.tolerate_missing_devices and hardware_init_error is not None:
                    now = time.monotonic()
                    if now - last_hw_retry >= hw_retry_interval_s:
                        last_hw_retry = now
                        try:
                            hardware = hardware_from_config()
                            hardware_init_error = None
                            print(f"[inference] Hardware reconnected: {type(hardware).__name__}")
                        except Exception as exc:
                            print(f"[inference] Still waiting for hardware: {exc}")

                t_loop = time.monotonic()
                ok, frame = cam.read()
                t_cap = time.monotonic()
                if not ok or frame is None:
                    print("[WARN] Failed to read frame — retrying …")
                    time.sleep(0.05)
                    continue

                frame_count += 1

                # 1. Detect
                detections = detector.detect(frame)
                t_yolo = time.monotonic()

                # 2. Track
                track = tracker.update(
                    detections,
                    frame.shape,
                    apple_label=args.apple_label,
                )

                # 3. Control (vision + live ESP telemetry when sub stack is active)
                telemetry = None
                if use_sub:
                    from src.telemetry_context import TelemetryContext
                    telemetry = TelemetryContext.from_sub_state()
                output = controller.compute(track, telemetry=telemetry)
                if args.web or use_sub:
                    from dataclasses import replace
                    from src.control_source import get_current_sdt
                    sdt = get_current_sdt(
                        SDT(output.steering_servo, output.drive_motor, output.camera_tilt_servo)
                    )
                    output = replace(
                        output,
                        steering_servo=sdt.s,
                        drive_motor=sdt.d,
                        camera_tilt_servo=sdt.t,
                    )

                # 3b. Send to hardware / sub bridge (SubBridgeOutput updates sub_state)
                try:
                    hardware.apply(output)
                except Exception as exc:
                    if args.tolerate_missing_devices:
                        hardware_init_error = exc
                        hardware = StubOutput({})
                        # Keep outputs stuck in manual while HW is missing.
                        set_mode("manual")
                        print(f"[WARN] Hardware apply failed; using stub: {exc}")
                    else:
                        raise
                if use_sub and frame_count % 30 == 0:
                    esp = "ESP OK" if telemetry and telemetry.fresh else "ESP --"
                    bat_s = (
                        f"{telemetry.battery_v:.1f}V"
                        if telemetry and telemetry.battery_v is not None
                        else "--"
                    )
                    dep_s = (
                        f"{telemetry.depth_m:.2f}m"
                        if telemetry and telemetry.depth_m is not None
                        else "--"
                    )
                    print(
                        f"[telemetry] {esp}  battery={bat_s}  depth={dep_s}  "
                        f"prox={output.proximity_t:.2f}  drive={output.drive_motor:+.2f}"
                        + (f"  ({output.approach_note})" if output.approach_note else "")
                    )

                # 4. Print
                if not quiet_terminal and (
                    not args.print_on_detect or output.apple_detected
                ):
                    if frame_count > 1:
                        sys.stdout.write("\033[13A\033[J")
                    print(output.pretty())
                    sys.stdout.flush()

                # 5. Display
                annotated = display.draw(frame, output)
                t_draw = time.monotonic()
                if serve_http:
                    set_latest_frame(annotated)
                if not display.show(annotated):
                    break   # user pressed q

                if args.timing and frame_count % 30 == 0:
                    total_ms = (t_draw - t_loop) * 1000
                    cap_ms = (t_cap - t_loop) * 1000
                    yolo_ms = (t_yolo - t_cap) * 1000
                    draw_ms = (t_draw - t_yolo) * 1000
                    loop_fps = display.fps
                    print(
                        f"[perf] cap={cap_ms:.0f}ms yolo={yolo_ms:.0f}ms "
                        f"ctrl+draw={draw_ms:.0f}ms total={total_ms:.0f}ms "
                        f"hud_fps={loop_fps:.1f}"
                    )

    finally:
        if cam is not None:
            cam.release()
        display.close()
        hardware.close()
        if esp_bridge is not None:
            esp_bridge.stop()

        elapsed = time.monotonic() - loop_start
        if manual_only:
            duration = time.monotonic() - manual_loop_start
            print(f"\n[inference] Manual-only mode ended after {duration:.1f}s")
        else:
            avg_fps = frame_count / elapsed if elapsed > 0 else 0
            print(f"\n[inference] Processed {frame_count} frames in "
                  f"{elapsed:.1f}s  ({avg_fps:.1f} fps avg)")


if __name__ == "__main__":
    main()
