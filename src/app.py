"""
app.py
------
Single entry point for the sub stack: dashboard, cameras, ESP, GPS, Xbox,
and optional YOLO (started from /sub/ or with --yolo).

``~/sub`` (``run.py``), ``sub_server.py``, and ``inference.py`` all call ``main()``.
"""

from __future__ import annotations

import argparse
import os
import signal
import sys
from pathlib import Path
from typing import Optional

PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

# HailoRT / OpenCV / Ultralytics otherwise dump INFO on every infer.
os.environ.setdefault("HAILORT_LOGGER_PATH", "NONE")
os.environ.setdefault("HAILORT_CONSOLE_LOGGER_LEVEL", "error")
os.environ.setdefault("OPENCV_LOG_LEVEL", "ERROR")
os.environ.setdefault("YOLO_VERBOSE", "False")


def parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Sub stack: dashboard + stereo cameras + ESP/GPS/Xbox. "
            "YOLO stays off until you pick a backend on /sub/, or pass --yolo."
        )
    )
    web = p.add_argument_group("web / dashboard")
    web.add_argument("--host", "--web-host", dest="host", default="0.0.0.0",
                     help="Bind address (default 0.0.0.0)")
    web.add_argument("--port", "--web-port", dest="port", type=int, default=8080,
                     help="HTTP port (default 8080)")
    web.add_argument("--web", action="store_true",
                     help="Accepted for compatibility; the dashboard always runs")

    ser = p.add_argument_group("peripherals")
    ser.add_argument("--serial-port", default=None,
                     help="ESP32 serial port override (default: config/hardware.yaml)")
    ser.add_argument("--no-esp", action="store_true", help="Skip ESP serial bridge")
    ser.add_argument("--no-xbox", action="store_true", help="Skip Xbox polling")
    ser.add_argument("--no-gps", action="store_true", help="Skip USB GPS auto-scan")
    ser.add_argument("--sub", action="store_true",
                     help="Accepted for compatibility; dashboard + ESP already start unless --no-sub")
    ser.add_argument("--no-sub", action="store_true",
                     help="Skip ESP/GPS/Xbox (cameras + dashboard only)")

    cam = p.add_argument_group("cameras")
    cam.add_argument("--no-camera", action="store_true", help="Skip USB cameras")
    cam.add_argument("--device", "--camera-device", dest="camera_device", default=None,
                     help="Preferred mono / left camera index or /dev/videoN")
    cam.add_argument("--left-device", default=None,
                     help="Left stereo camera (default: cameras.left_device)")
    cam.add_argument("--right-device", default=None,
                     help="Right stereo camera (default: cameras.right_device)")
    cam.add_argument("--stereo", action="store_true",
                     help="Force two-camera stereo even if num_cameras is 1")
    cam.add_argument("--no-stereo", "--no-stero", action="store_true",
                     help="Force single-camera preview")
    cam.add_argument("--width", type=int, default=640, help="Stereo/YOLO capture width")
    cam.add_argument("--height", type=int, default=480, help="Stereo/YOLO capture height")
    cam.add_argument("--fov-width", type=int, default=None,
                     help="FOV width (0 or omit = auto max MJPEG from v4l2)")
    cam.add_argument("--fov-height", type=int, default=None,
                     help="FOV height (0 or omit = auto max MJPEG from v4l2)")

    yolo = p.add_argument_group("YOLO")
    yolo.add_argument("--yolo", action="store_true",
                      help="Start inference immediately (same as picking a backend on /sub/)")
    yolo.add_argument("--model", default=None,
                      help="Catalog id to load with --yolo (default: active_model)")
    yolo.add_argument("--backend", choices=["pytorch", "ncnn", "openvino", "hailo"],
                      default=None, help="Override backend (default: config/model.yaml)")
    yolo.add_argument("--imgsz", type=int, default=None, help="Override model input size")
    yolo.add_argument("--tracker-strategy", choices=["best_confidence", "closest_to_centre"],
                      default="best_confidence")
    yolo.add_argument("--control-profile", choices=["config", "stable", "aggressive"],
                      default="config")
    yolo.add_argument("--timing", action="store_true",
                      help="Log loop FPS once every 30 seconds")
    yolo.add_argument("--print-on-detect", action="store_true",
                      help="Print ControlOutput when a target is detected")
    yolo.add_argument("--quiet", action="store_true", help="Less console output")
    yolo.add_argument("--headless", action="store_true",
                      help="Accepted for compatibility (dashboard has no OpenCV window)")
    yolo.add_argument("--tolerate-missing-devices", action="store_true",
                      help="Accepted for compatibility; cameras/ESP already retry")
    return p.parse_args(argv)


def _print_urls(host: str, port: int) -> None:
    print(f"[run] Dashboard: http://localhost:{port}/sub/")
    print(f"[run] Stream:    http://localhost:{port}/")
    if host != "0.0.0.0":
        if host not in ("127.0.0.1", "localhost") and not str(host).startswith("127."):
            print(f"[run] Dashboard (LAN): http://{host}:{port}/sub/")
        return
    for ip in _lan_ipv4_addresses():
        print(f"[run] Dashboard (LAN): http://{ip}:{port}/sub/")
    try:
        import subprocess
        r = subprocess.run(
            ["tailscale", "ip", "-4"],
            capture_output=True, text=True, timeout=2,
        )
        if r.returncode == 0 and r.stdout.strip():
            ts = r.stdout.strip().split()[0]
            if ts and not ts.startswith("127."):
                print(f"[run] Dashboard (Tailscale): http://{ts}:{port}/sub/")
    except Exception:
        pass


def _is_usable_lan_ip(ip: str) -> bool:
    """Skip loopback / placeholder addresses (Pi /etc/hosts often has 127.0.1.1)."""
    if not ip or ip.startswith("127.") or ip.startswith("0.") or ip.startswith("169.254."):
        return False
    return True


def _lan_ipv4_addresses() -> list[str]:
    import socket

    found: list[str] = []

    def _add(ip: str) -> None:
        if _is_usable_lan_ip(ip) and ip not in found:
            found.append(ip)

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.connect(("1.1.1.1", 80))
            _add(probe.getsockname()[0])
        finally:
            probe.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            _add(info[4][0])
    except OSError:
        pass
    return found


def _resolve_yolo_target(args: argparse.Namespace) -> tuple[str, str]:
    from src.model_runtime import _load_yaml, discover_models

    cfg = _load_yaml()
    model_id = str(args.model or cfg.get("active_model") or "detect").strip()
    backend = str(args.backend or cfg.get("backend") or "hailo").strip().lower()
    models = {m["id"]: m for m in discover_models()}
    entry = models.get(model_id)
    if entry is None and models:
        main = next((m for m in models.values() if m.get("main")), None)
        entry = main or next(iter(models.values()))
        model_id = entry["id"]
        print(f"[run] Unknown model; using {model_id!r}")
    backends = (entry or {}).get("backends") or {}
    be = backends.get(backend) or {}
    if not be.get("available"):
        for name in ("hailo", "ncnn", "pytorch", "openvino"):
            info = backends.get(name) or {}
            if info.get("available"):
                print(f"[run] backend {backend!r} unavailable; using {name}")
                backend = name
                break
    return model_id, backend


def main(argv: Optional[list[str]] = None) -> None:
    args = parse_args(argv)

    try:
        from src.web_stream import _get_app, register_sub_dashboard, run_server
    except ImportError as e:
        if "flask" in str(e).lower():
            print("[ERROR] Flask required. pip install flask")
        else:
            print(f"[ERROR] {e}")
        sys.exit(1)

    _get_app()
    register_sub_dashboard(start_services=False)

    skip_peripherals = bool(args.no_sub)
    esp_port = ""
    if not skip_peripherals and not args.no_esp:
        from src.esp_bridge import get_esp_bridge
        bridge = get_esp_bridge(port=args.serial_port, autostart=False)
        bridge.start()
        print(f"[run] ESP bridge on {bridge.port}")
        esp_port = bridge.port
    elif args.no_esp or skip_peripherals:
        print("[run] ESP bridge skipped")

    if not skip_peripherals and not args.no_gps:
        from src.gps_reader import connect_gps, is_gps_enabled
        if is_gps_enabled():
            connect_gps(esp_port=esp_port)
            print("[run] GPS auto-scan started")
        else:
            print("[run] GPS disabled in config/hardware.yaml")
    elif args.no_gps or skip_peripherals:
        print("[run] GPS skipped")

    if not skip_peripherals and not args.no_xbox:
        try:
            from src.xbox_controller import connect_xbox, is_xbox_enabled
            if is_xbox_enabled():
                connect_xbox()
                print("[run] Xbox polling started")
            else:
                print("[run] Xbox disabled in config/hardware.yaml")
        except ImportError:
            print("[WARN] Xbox unavailable — install pygame")
    elif args.no_xbox or skip_peripherals:
        print("[run] Xbox skipped")

    from src.inference_service import get_inference_service

    yolo = get_inference_service()
    yolo.configure(
        timing=args.timing,
        print_on_detect=args.print_on_detect,
        quiet=args.quiet,
        imgsz=args.imgsz,
        control_profile=args.control_profile,
        tracker_strategy=args.tracker_strategy,
        width=args.width,
        height=args.height,
        fov_width=args.fov_width,
        fov_height=args.fov_height,
        left_device=args.left_device or args.camera_device,
        right_device=args.right_device,
        force_stereo=bool(args.stereo),
        force_mono=bool(args.no_stereo),
    )
    if not args.no_camera:
        preview_dev = args.left_device or args.camera_device or 0
        yolo.enable_idle_preview(preview_dev)
    else:
        print("[run] Cameras skipped (--no-camera)")

    if args.yolo:
        model_id, backend = _resolve_yolo_target(args)
        print(f"[run] Starting YOLO model={model_id!r} backend={backend}")
        result = yolo.start(model_id, backend)
        if not result.get("ok"):
            print(f"[run] YOLO start failed: {result.get('error')}")
    else:
        print("[run] YOLO off — pick a backend on /sub/, or restart with --yolo")

    shutting_down = False

    def _shutdown(sig, frame):
        # Do not join GPS/ESP/serial here — pyserial close can deadlock
        # in a signal handler, leaving :8080 bound and the browser hanging.
        nonlocal shutting_down
        if shutting_down:
            os._exit(1)
        shutting_down = True
        print("\n[run] Shutting down …", flush=True)
        os._exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    _print_urls(args.host, args.port)
    print("[run] Press Ctrl+C to stop.")
    run_server(host=args.host, port=args.port)


if __name__ == "__main__":
    main()
