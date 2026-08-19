"""
inference.py
------------
Main inference loop.  Run this on the Raspberry Pi 5.

What it does each frame
-----------------------
1.  Capture frame(s) from USB camera(s)  (src/camera.py)
2.  Run YOLO detection (one or two models)  (src/detector.py)
3.  Pick best apple + compute error (src/tracker.py)
4.  Optional stereo range from bbox centres (src/stereo.py)
5.  Compute servo/motor commands   (src/controller.py)
6.  Print ControlOutput table      (controller.ControlOutput.pretty())
7.  Draw annotated camera view     (src/display.py)

Usage
-----
    # With display window (requires X forwarding or local monitor)
    python inference.py

    # Headless (SSH without X forwarding — prints to terminal only)
    python inference.py --headless

    # Different camera device
    python inference.py --device 1

    # Two-camera stereo (also the default when cameras.num_cameras: 2)
    python inference.py --web --timing --left-device 0 --right-device 2

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
from collections import deque
from pathlib import Path

PROJECT_ROOT = Path(__file__).parent
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="YOLO apple detection inference on Raspberry Pi 5."
    )
    parser.add_argument(
        "--device", type=int, default=None,
        help="USB camera device index (mono mode; default: 0, or cameras.left_device)",
    )
    parser.add_argument(
        "--stereo",
        action="store_true",
        help="Force two-camera stereo (overrides cameras.num_cameras)",
    )
    parser.add_argument(
        "--no-stereo",
        action="store_true",
        help="Force single-camera mode even when cameras.num_cameras is 2",
    )
    parser.add_argument(
        "--left-device",
        default=None,
        help="Left stereo camera index or /dev/videoN (default: cameras.left_device)",
    )
    parser.add_argument(
        "--right-device",
        default=None,
        help="Right stereo camera index or /dev/videoN (default: cameras.right_device)",
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
        "--backend",
        choices=["pytorch", "ncnn", "openvino", "hailo"],
        default=None,
        help="Override inference backend from config/model.yaml (hailo = Hailo-8L HAT)",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=None,
        help="Override model input size from config/model.yaml (320 is faster on Pi)",
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
        "--no-gps",
        action="store_true",
        help="With --sub, skip USB GPS auto-scan",
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


def _quiet_third_party_logs() -> None:
    """Stop Flask/Ultralytics/OpenCV from printing on every request/frame."""
    import logging
    import os

    os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    logging.getLogger("ultralytics").setLevel(logging.ERROR)


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


def _parse_device(value) -> int | str:
    if value is None:
        return 0
    if isinstance(value, int):
        return value
    text = str(value).strip()
    if text.lstrip("-").isdigit():
        return int(text)
    return text


def _stereo_enabled(args: argparse.Namespace, stereo_cfg) -> bool:
    if args.no_stereo:
        return False
    if args.stereo:
        return True
    return bool(stereo_cfg.enabled)


def _output_for_view(output, track):
    """Copy actuator/range fields onto a per-camera bbox for drawing."""
    from dataclasses import replace
    from src.tracker import TrackResult

    if not isinstance(track, TrackResult) or not track.apple_detected:
        return replace(
            output,
            apple_detected=False,
            target_x=0,
            target_y=0,
            bbox_x1=0,
            bbox_y1=0,
            bbox_x2=0,
            bbox_y2=0,
            confidence=0.0,
        )
    return replace(
        output,
        apple_detected=True,
        target_x=track.target_x,
        target_y=track.target_y,
        bbox_x1=track.bbox_x1,
        bbox_y1=track.bbox_y1,
        bbox_x2=track.bbox_x2,
        bbox_y2=track.bbox_y2,
        bbox_width=track.bbox_width,
        bbox_height=track.bbox_height,
        bbox_area=track.bbox_area,
        frame_area=track.frame_area,
        confidence=track.confidence,
        error_x=track.error_x,
        error_y=track.error_y,
    )


def _compose_stereo_view(left_drawn, right_drawn, rig_reversed: bool = False):
    """Side-by-side L|R; flip panel order when config labels ≠ physical rig."""
    from src.stereo import compose_side_by_side

    if rig_reversed:
        return compose_side_by_side(right_drawn, left_drawn)
    return compose_side_by_side(left_drawn, right_drawn)


def _start_web_preview(
    *,
    grabber,
    grabber_right,
    display,
    use_stereo: bool,
    hud_holder: dict,
    stop_event: threading.Event,
) -> None:
    """Publish L/R camera frames as soon as they arrive; overlay last YOLO boxes."""
    from src.web_stream import set_latest_frame

    def _loop() -> None:
        while not stop_event.is_set():
            t0 = time.monotonic()
            ok_l, left = grabber.peek() if grabber is not None else (False, None)
            if not ok_l or left is None:
                stop_event.wait(0.02)
                continue
            ok_r, right = (False, None)
            if use_stereo and grabber_right is not None:
                ok_r, right = grabber_right.peek()
            with hud_holder["lock"]:
                output = hud_holder.get("output")
                track_left = hud_holder.get("track_left")
                track_right = hud_holder.get("track_right")
                rig_reversed = bool(hud_holder.get("rig_reversed"))
                loop_fps = hud_holder.get("loop_fps")
            if output is not None and track_left is not None:
                left = display.draw(
                    left,
                    _output_for_view(output, track_left),
                    count_fps=False,
                    copy=False,
                    overlay_fps=loop_fps,
                )
            elif output is not None:
                left = display.draw(
                    left,
                    output,
                    count_fps=False,
                    copy=False,
                    overlay_fps=loop_fps,
                )
            if use_stereo and ok_r and right is not None:
                if output is not None and track_right is not None:
                    right = display.draw(
                        right,
                        _output_for_view(output, track_right),
                        count_fps=False,
                        copy=False,
                        hud=False,
                        gauges=False,
                    )
                elif output is not None:
                    right = display.draw(
                        right,
                        output,
                        count_fps=False,
                        copy=False,
                        hud=False,
                        gauges=False,
                    )
                left = _compose_stereo_view(left, right, rig_reversed)
            set_latest_frame(left)
            # Preview is JPEG/HUD only; cap so it does not starve Hailo.
            stop_event.wait(max(0.0, 0.066 - (time.monotonic() - t0)))

    threading.Thread(target=_loop, name="web-preview", daemon=True).start()


def _start_sub_stack(args: argparse.Namespace, serial_port: str | None) -> object | None:
    """Start ESP bridge, GPS, optional Xbox polling, and /sub/ routes. Returns bridge or None."""
    from src.esp_bridge import get_esp_bridge
    from src.sub_state import get_sub_state

    bridge = get_esp_bridge(port=serial_port, autostart=False)
    bridge.start()
    print(f"[inference] ESP bridge on {bridge.port} (telemetry + S2/B actuators)")

    if not getattr(args, "no_gps", False):
        from src.gps_reader import connect_gps, is_gps_enabled
        if is_gps_enabled():
            connect_gps(esp_port=bridge.port)
            print("[inference] GPS auto-scan started (plug in USB GPS anytime)")
        else:
            print("[inference] GPS disabled in config/hardware.yaml (gps.enabled: false)")

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
    _quiet_third_party_logs()

    # ------------------------------------------------------------------
    # Import modules (deferred so path is set up first)
    # ------------------------------------------------------------------
    from src.camera import (
        Camera,
        CameraError,
        FrameGrabber,
        assign_camera_device,
        format_usb_camera_report,
    )

    try:
        import cv2
        cv2.setNumThreads(1)
    except Exception:
        pass
    from src.model_runtime import init_model_runtime
    from src.tracker import Tracker
    from src.controller import Controller
    from src.display import Display
    from src.stereo import (
        fuse_tracks,
        load_stereo_config,
        pair_tracks,
        triangulate_with_swap,
    )

    hw_config = _load_hw_config()
    stereo_cfg = load_stereo_config()
    use_stereo = _stereo_enabled(args, stereo_cfg)
    left_device = _parse_device(
        args.left_device
        if args.left_device is not None
        else (args.device if args.device is not None else stereo_cfg.left_device)
    )
    right_device = _parse_device(
        args.right_device if args.right_device is not None else stereo_cfg.right_device
    )
    mono_device = _parse_device(
        args.device if args.device is not None else stereo_cfg.left_device
    )
    print(format_usb_camera_report())
    if use_stereo:
        left_device, left_note = assign_camera_device(left_device)
        right_device, right_note = assign_camera_device(
            right_device, exclude=(str(left_device),)
        )
        for note in (left_note, right_note):
            if note:
                print(f"[inference] {note}")
    else:
        mono_device, mono_note = assign_camera_device(mono_device)
        if mono_note:
            print(f"[inference] {mono_note}")

    # ------------------------------------------------------------------
    # Open cameras first so we only load a second model if both exist
    # ------------------------------------------------------------------
    cam = None
    cam_right = None
    grabber = None
    grabber_right = None
    detect_pool = None
    manual_only = False
    try:
        if use_stereo:
            print(
                f"[inference] Stereo requested  baseline={stereo_cfg.baseline_m * 100:.1f}cm  "
                f"fov={stereo_cfg.fov_h_deg:g}°"
                f"{' diagonal' if stereo_cfg.fov_is_diagonal else ' horizontal'}  "
                f"left={left_device}  right={right_device}"
            )
            print(f"[inference] Opening left camera {left_device} …")
            cam = Camera(device=left_device, width=args.width, height=args.height)
            cam.open()
            print(f"[inference] Opening right camera {right_device} …")
            try:
                cam_right = Camera(device=right_device, width=args.width, height=args.height)
                cam_right.open()
            except CameraError as right_exc:
                print(f"[WARN] Right camera failed: {right_exc}")
                print("[WARN] Falling back to single-camera mode (no stereo range).")
                print(format_usb_camera_report())
                use_stereo = False
                cam_right = None
        else:
            print(f"[inference] Opening camera device {mono_device} …")
            cam = Camera(device=mono_device, width=args.width, height=args.height)
            cam.open()
    except CameraError as exc:
        if args.tolerate_missing_devices:
            manual_only = True
            print(f"[WARN] Camera init failed; skipping YOLO loop (manual-only): {exc}")
        else:
            print(f"[ERROR] {exc}")
            print(format_usb_camera_report())
            sys.exit(1)

    grabber = FrameGrabber(cam) if cam is not None else None
    grabber_right = FrameGrabber(cam_right) if cam_right is not None else None
    if grabber is not None:
        grabber.start()
    if grabber_right is not None:
        grabber_right.start()
    if grabber is not None:
        print("[inference] Async capture started (latest-frame, drop stale)")
    if cam is not None and cam_right is not None:
        ln, rn = cam.native_size, cam_right.native_size
        print(
            f"[inference] Stereo working size {cam.width}x{cam.height}  "
            f"left native={ln[0]}x{ln[1]}  right native={rn[0]}x{rn[1]}"
        )
        if ln != rn:
            print(
                "[inference] Native resolutions differ — both streams are resized "
                f"to {cam.width}x{cam.height} before YOLO/stereo"
            )

    # ------------------------------------------------------------------
    # Initialise components
    # ------------------------------------------------------------------
    print("[inference] Initialising detector …")
    try:
        from src.model_runtime import init_model_runtime

        model_runtime = init_model_runtime(
            backend=args.backend,
            imgsz_override=args.imgsz,
        )
        if use_stereo:
            print("[inference] Stereo uses one shared NCNN model (avoids CPU oversubscribe)")
        print(
            f"[inference] YOLO model={model_runtime.active_id!r} "
            f"track_label={model_runtime.track_label!r}"
        )
    except Exception as exc:
        print(f"[ERROR] Could not load model: {exc}")
        sys.exit(1)

    tracker = Tracker(strategy=args.tracker_strategy)
    tracker_right = Tracker(strategy=args.tracker_strategy) if use_stereo else None
    detect_pool = None
    controller = Controller.from_hardware_config(profile=args.control_profile)
    if args.control_profile != "config":
        print(f"[inference] Control profile override: {args.control_profile}")

    from src.hardware import from_config as hardware_from_config
    from src.hardware import StubOutput

    use_sub = _sub_stack_enabled(args, hw_config)
    serve_http = args.web or use_sub
    # Browser/sub dashboard mode: skip OpenCV window (imshow over VNC/SSH kills FPS).
    display = Display(headless=args.headless or serve_http)
    quiet_terminal = args.quiet or serve_http
    esp_bridge = None
    preview_stop = threading.Event()
    hud_holder: dict = {
        "lock": threading.Lock(),
        "output": None,
        "track_left": None,
        "track_right": None,
        "rig_reversed": False,
        "loop_fps": 0.0,
    }
    loop_stamps: deque[float] = deque(maxlen=30)
    stereo_swap_streak = 0
    stereo_rig_reversed = False

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
            from src.web_stream import _get_app, run_server, register_sub_dashboard
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
        if grabber is not None:
            _start_web_preview(
                grabber=grabber,
                grabber_right=grabber_right,
                display=display,
                use_stereo=use_stereo,
                hud_holder=hud_holder,
                stop_event=preview_stop,
            )
            print("[inference] Web preview = latest camera frame (does not wait for YOLO)")
    elif use_sub:
        # Headless YOLO + ESP telemetry/actuators (no browser UI).
        esp_bridge = _start_sub_stack(args, args.serial_port)

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
                if grabber is not None:
                    ok, frame = grabber.peek()
                    if use_stereo and grabber_right is not None:
                        ok_right, frame_right = grabber_right.peek()
                    else:
                        ok_right, frame_right = True, None
                else:
                    ok, frame = cam.read()
                    ok_right, frame_right = True, None
                    if use_stereo and cam_right is not None:
                        ok_right, frame_right = cam_right.read()
                t_cap = time.monotonic()
                if not ok or frame is None:
                    time.sleep(0.005)
                    continue
                if use_stereo:
                    if not ok_right or frame_right is None:
                        time.sleep(0.005)
                        continue

                frame_count += 1
                loop_stamps.append(time.monotonic())
                if len(loop_stamps) >= 2:
                    loop_fps = (len(loop_stamps) - 1) / (
                        loop_stamps[-1] - loop_stamps[0]
                    )
                else:
                    loop_fps = 0.0

                # 1. Detect — one shared model (sequential). Two NCNN nets in
                # parallel oversubscribe the Pi and are usually slower.
                detections = model_runtime.detect(frame)
                detections_right = (
                    model_runtime.detect(frame_right)
                    if use_stereo and frame_right is not None
                    else []
                )
                track_label = model_runtime.track_label
                t_yolo = time.monotonic()

                # 2. Track
                track_left = tracker.update(
                    detections,
                    frame.shape,
                    apple_label=track_label,
                )
                track = track_left
                track_right = None
                stereo_result = None
                if use_stereo and tracker_right is not None and frame_right is not None:
                    track_right = tracker_right.update(
                        detections_right,
                        frame_right.shape,
                        apple_label=track_label,
                    )
                    stereo_result, stereo_swapped = triangulate_with_swap(
                        track_left,
                        track_right,
                        frame.shape[1],
                        frame.shape[0],
                        stereo_cfg,
                    )
                    if stereo_swapped:
                        stereo_swap_streak += 1
                        if stereo_swap_streak >= 5 and not stereo_rig_reversed:
                            stereo_rig_reversed = True
                            hud_holder["rig_reversed"] = True
                            print(
                                "[stereo] Config left/right labels appear reversed — "
                                "flipping side-by-side display (swap left_device/right_device "
                                "in hardware.yaml to fix permanently)"
                            )
                    else:
                        stereo_swap_streak = 0
                    paired = pair_tracks(
                        track_left, track_right, stereo_cfg.match_max_dy_px
                    )
                    track = fuse_tracks(track_left, track_right, paired)

                # 3. Control (vision + live ESP telemetry when sub stack is active)
                telemetry = None
                if use_sub:
                    from src.telemetry_context import TelemetryContext
                    telemetry = TelemetryContext.from_sub_state()
                if stereo_result is not None:
                    output = controller.compute(
                        track,
                        telemetry=telemetry,
                        range_m=stereo_result.range_m if stereo_result.ok else None,
                        stereo_ok=stereo_result.ok,
                        stereo_note="" if stereo_result.ok else stereo_result.reason,
                    )
                else:
                    output = controller.compute(track, telemetry=telemetry)
                if (
                    args.timing
                    and frame_count % 30 == 0
                    and stereo_result is not None
                    and stereo_result.ok
                ):
                    print(
                        f"[stereo] range={stereo_result.range_m:.2f}m  "
                        f"z={stereo_result.z_m:.2f}m  "
                        f"d={stereo_result.disparity_px:.1f}px"
                    )
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
                if use_sub and args.timing and frame_count % 30 == 0:
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

                # 5. Display — web preview thread owns the JPEG; do not wait on YOLO.
                t_draw = time.monotonic()
                if serve_http:
                    with hud_holder["lock"]:
                        hud_holder["output"] = output
                        hud_holder["track_left"] = track_left
                        hud_holder["track_right"] = track_right
                        hud_holder["loop_fps"] = loop_fps
                else:
                    if use_stereo and frame_right is not None and track_right is not None:
                        annotated = _compose_stereo_view(
                            display.draw(
                                frame,
                                _output_for_view(output, track_left),
                                copy=False,
                            ),
                            display.draw(
                                frame_right,
                                _output_for_view(output, track_right),
                                count_fps=False,
                                copy=False,
                            ),
                            stereo_rig_reversed,
                        )
                    else:
                        annotated = display.draw(frame, output, copy=False)
                    t_draw = time.monotonic()
                    if not display.show(annotated):
                        break   # user pressed q

                if args.timing and frame_count % 30 == 0:
                    total_ms = (t_draw - t_loop) * 1000
                    cap_ms = (t_cap - t_loop) * 1000
                    yolo_ms = (t_yolo - t_cap) * 1000
                    draw_ms = (t_draw - t_yolo) * 1000
                    loop_fps = (
                        (len(loop_stamps) - 1)
                        / (loop_stamps[-1] - loop_stamps[0])
                        if len(loop_stamps) >= 2
                        else 0.0
                    )
                    print(
                        f"[perf] cap={cap_ms:.0f}ms yolo={yolo_ms:.0f}ms "
                        f"ctrl+draw={draw_ms:.0f}ms total={total_ms:.0f}ms "
                        f"loop_fps={loop_fps:.1f}"
                    )

    finally:
        preview_stop.set()
        if grabber is not None:
            grabber.stop()
        if grabber_right is not None:
            grabber_right.stop()
        if cam is not None:
            cam.release()
        if cam_right is not None:
            cam_right.release()
        if detect_pool is not None:
            detect_pool.shutdown(wait=False)
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
