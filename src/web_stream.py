"""
web_stream.py
-------------
Serves live camera JPEG streams over HTTP.

Channels:
  fov        — pilot / wide FOV camera (main HUD)
  yolo_left  — stereo left with detection overlay (lazy-loaded in dashboard)
  yolo_right — stereo right with detection overlay

Legacy ``/snapshot`` maps to ``fov``. Sub dashboard uses ``/snapshot/<channel>``.
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

_app = None
_CHANNELS = ("fov", "yolo_left", "yolo_right")
_frame_lock = threading.Lock()
_frame_cond = threading.Condition(_frame_lock)
_encode_event = threading.Event()
_encoder_started = False
_encode_busy = False
_hud_painter = None
_JPEG_QUALITY = 50
_FOV_JPEG_QUALITY = 82
_STREAM_MAX_WIDTH = 1280
_YOLO_MAX_WIDTH = 640
_LIVE_JPEG_JS = Path(__file__).with_name("live_jpeg.js")
_NO_STORE = {
    "Cache-Control": "no-store, no-cache, must-revalidate, max-age=0",
    "Pragma": "no-cache",
    "Expires": "0",
    "X-Accel-Buffering": "no",
}
_CHANNEL_ALIASES = {
    "": "fov",
    "fov": "fov",
    "main": "fov",
    "pilot": "fov",
    "yolo_left": "yolo_left",
    "left": "yolo_left",
    "yolo_right": "yolo_right",
    "right": "yolo_right",
}
# Encode YOLO side streams only while a client is polling (saves CPU).
_yolo_client_until: dict[str, float] = {"yolo_left": 0.0, "yolo_right": 0.0}
_YOLO_CLIENT_HOLD_S = 8.0

_PLACEHOLDER_JPEG: Optional[bytes] = None


def _empty_channel() -> dict:
    return {
        "bytes": None,
        "pending": None,
        "hud": None,
        "overlay_fps": None,
        "gen": 0,
        "max_width": _STREAM_MAX_WIDTH,
    }


_frame_stores: dict[str, dict] = {ch: _empty_channel() for ch in _CHANNELS}
_frame_stores["fov"]["jpeg_quality"] = _FOV_JPEG_QUALITY
_frame_stores["fov"]["max_width"] = 0  # 0 = no downscale before HUD encode
_frame_stores["yolo_left"]["max_width"] = _YOLO_MAX_WIDTH
_frame_stores["yolo_right"]["max_width"] = _YOLO_MAX_WIDTH


def _normalize_channel(name: str | None) -> str:
    key = (name or "").strip().lower()
    ch = _CHANNEL_ALIASES.get(key, key)
    if ch not in _frame_stores:
        ch = "fov"
    return ch


def _get_placeholder_jpeg() -> bytes:
    global _PLACEHOLDER_JPEG
    if _PLACEHOLDER_JPEG is None:
        img = np.zeros((120, 400, 3), dtype=np.uint8)
        img[:] = (60, 60, 60)
        cv2.putText(
            img, "Waiting for camera...",
            (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (200, 200, 200), 2,
        )
        ok, buf = cv2.imencode(".jpg", img)
        if ok:
            _PLACEHOLDER_JPEG = buf.tobytes()
    return _PLACEHOLDER_JPEG or b""


def touch_yolo_client(channel: str) -> None:
    ch = _normalize_channel(channel)
    if ch in _yolo_client_until:
        _yolo_client_until[ch] = time.monotonic() + _YOLO_CLIENT_HOLD_S


def yolo_channel_active(channel: str) -> bool:
    ch = _normalize_channel(channel)
    if ch not in _yolo_client_until:
        return False
    return time.monotonic() < _yolo_client_until[ch]


def encoder_busy() -> bool:
    if _encode_busy:
        return True
    with _frame_lock:
        for ch in _CHANNELS:
            if _frame_stores[ch].get("pending") is not None:
                return True
    return False


def set_latest_frame(
    frame_bgr: np.ndarray,
    *,
    channel: str = "fov",
    hud=None,
    overlay_fps=None,
) -> None:
    """Queue a frame for background JPEG encode (does not block capture loop)."""
    if frame_bgr is None or frame_bgr.size == 0:
        return
    ch = _normalize_channel(channel)
    if ch.startswith("yolo_") and not yolo_channel_active(ch):
        return
    app = _get_app()
    if hasattr(app, "set_latest_frame"):
        app.set_latest_frame(frame_bgr, channel=ch, hud=hud, overlay_fps=overlay_fps)


def _get_app():
    global _app
    if _app is None:
        from flask import Flask, Response, request

        _app = Flask(__name__)

        def set_latest_frame_bgr(
            frame_bgr: np.ndarray,
            *,
            channel: str = "fov",
            hud=None,
            overlay_fps=None,
        ) -> None:
            if frame_bgr is None or frame_bgr.size == 0:
                return
            ch = _normalize_channel(channel)
            if ch.startswith("yolo_") and not yolo_channel_active(ch):
                return
            with _frame_lock:
                store = _frame_stores[ch]
                store["pending"] = frame_bgr
                store["hud"] = hud
                store["overlay_fps"] = overlay_fps
            _encode_event.set()
            _ensure_encoder()

        def generate_feed():
            boundary = "frame"
            while True:
                with _frame_lock:
                    frame_bytes = _frame_stores["fov"]["bytes"]
                if not frame_bytes:
                    frame_bytes = _get_placeholder_jpeg()
                yield (
                    b"--" + boundary.encode() + b"\r\n"
                    b"Content-Type: image/jpeg\r\n"
                    b"Content-Length: " + str(len(frame_bytes)).encode() + b"\r\n\r\n"
                    + frame_bytes + b"\r\n"
                )
                time.sleep(0.01)

        def _snapshot_response(channel: str):
            ch = _normalize_channel(channel)
            if ch.startswith("yolo_"):
                touch_yolo_client(ch)
            since = request.args.get("since", default=0, type=int) or 0
            with _frame_cond:
                store = _frame_stores[ch]
                if store["bytes"] is not None and store["gen"] <= since:
                    _frame_cond.wait_for(
                        lambda: _frame_stores[ch]["gen"] > since,
                        timeout=0.25,
                    )
                frame_bytes = _frame_stores[ch]["bytes"]
                gen = store["gen"]
            if not frame_bytes:
                frame_bytes = _get_placeholder_jpeg()
            headers = dict(_NO_STORE)
            headers["X-Frame-Gen"] = str(gen)
            headers["X-Stream-Channel"] = ch
            return Response(
                frame_bytes,
                mimetype="image/jpeg",
                headers=headers,
            )

        @_app.route("/")
        def index():
            return """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Apple RC Sub — Live</title>
  <style>
    body { margin: 0; background: #1a1a1a; color: #e0e0e0;
           font-family: system-ui, sans-serif; text-align: center; padding: 1rem; }
    h1 { font-size: 1.25rem; margin-bottom: 0.5rem; }
    p { color: #888; font-size: 0.875rem; margin-bottom: 1rem; }
    img, canvas { max-width: 100%; border-radius: 8px; background: #000; }
  </style>
</head>
<body>
  <h1>Apple RC Sub</h1>
  <p>FOV pilot feed. Dashboard: <a href="/sub/" style="color:#40916c">/sub/</a></p>
  <canvas id="cam" aria-label="Live stream"></canvas>
  <script src="/live_jpeg.js"></script>
  <script>attachLiveJpeg(document.getElementById('cam'), '/snapshot/fov');</script>
</body>
</html>
"""

        @_app.route("/video_feed")
        def video_feed():
            return Response(
                generate_feed(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
                headers=_NO_STORE,
            )

        @_app.route("/snapshot")
        @_app.route("/snapshot/<channel>")
        def snapshot(channel: str = ""):
            return _snapshot_response(channel)

        @_app.route("/live_jpeg.js")
        def live_jpeg_js():
            return Response(
                _LIVE_JPEG_JS.read_text(encoding="utf-8"),
                mimetype="application/javascript",
                headers=_NO_STORE,
            )

        _app.set_latest_frame = set_latest_frame_bgr
    return _app


def _encode_pending() -> None:
    global _hud_painter, _encode_busy
    while True:
        _encode_event.wait()
        _encode_event.clear()
        pending_jobs: list[tuple[str, np.ndarray, object, object, int]] = []
        with _frame_lock:
            for ch in _CHANNELS:
                store = _frame_stores[ch]
                frame = store.get("pending")
                if frame is None:
                    continue
                pending_jobs.append((
                    ch,
                    frame,
                    store.get("hud"),
                    store.get("overlay_fps"),
                    int(store.get("max_width") or _STREAM_MAX_WIDTH),
                ))
                store["pending"] = None
                store["hud"] = None
                store["overlay_fps"] = None
        if not pending_jobs:
            continue
        _encode_busy = True
        try:
            for ch, frame, hud, overlay_fps, max_w in pending_jobs:
                h, w = frame.shape[:2]
                if max_w > 0 and w > max_w:
                    scale = max_w / float(w)
                    frame = cv2.resize(
                        frame,
                        (max_w, max(1, int(h * scale))),
                        interpolation=cv2.INTER_AREA,
                    )
                if hud is not None and ch in ("yolo_left", "yolo_right"):
                    from src.display import Display

                    painter = _hud_painter
                    if painter is None:
                        painter = Display(headless=True)
                        _hud_painter = painter
                    painter.overlay_hud(
                        frame,
                        hud,
                        overlay_fps=overlay_fps,
                        gauges=True,
                        hud=True,
                        supersample=1,
                    )
                quality = int(
                    _frame_stores[ch].get("jpeg_quality") or _JPEG_QUALITY
                )
                ok, buf = cv2.imencode(
                    ".jpg",
                    frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), quality],
                )
                if ok:
                    with _frame_cond:
                        store = _frame_stores[ch]
                        store["bytes"] = buf.tobytes()
                        store["gen"] = int(store.get("gen") or 0) + 1
                        _frame_cond.notify_all()
        finally:
            _encode_busy = False


def _ensure_encoder() -> None:
    global _encoder_started
    if _encoder_started:
        return
    thread = threading.Thread(target=_encode_pending, name="mjpeg-encode", daemon=True)
    thread.start()
    _encoder_started = True


def register_sub_dashboard(start_services: bool = True) -> None:
    from src.sub_web import register_sub_dashboard as _register
    app = _get_app()
    _register(app, start_services=start_services)


def _pids_listening_on_port(port: int) -> list[int]:
    """Best-effort PIDs bound to TCP `port` (fuser / ss)."""
    import subprocess

    pids: list[int] = []

    def _collect(text: str) -> None:
        for tok in text.replace(",", " ").replace("pid=", " ").split():
            if not tok.isdigit():
                continue
            n = int(tok)
            if n == port or n in pids:
                continue
            if not Path(f"/proc/{n}").exists():
                continue
            pids.append(n)

    try:
        r = subprocess.run(
            ["fuser", f"{port}/tcp"],
            capture_output=True,
            text=True,
            timeout=1,
        )
        _collect((r.stdout or "") + " " + (r.stderr or ""))
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass
    if pids:
        return pids
    try:
        r = subprocess.run(
            ["ss", "-ltnp"],
            capture_output=True,
            text=True,
            timeout=1,
        )
        for line in (r.stdout or "").splitlines():
            if f":{port} " in line or line.endswith(f":{port}"):
                _collect(line)
    except (FileNotFoundError, subprocess.TimeoutExpired, OSError):
        pass
    return pids


def _cmd_for_pid(pid: int) -> str:
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(b"\x00", b" ").decode().strip()
    except OSError:
        return ""


def run_server(host: str = "0.0.0.0", port: int = 5000) -> None:
    import logging

    logging.getLogger("werkzeug").setLevel(logging.ERROR)
    try:
        from flask import cli as flask_cli
        flask_cli.show_server_banner = lambda *args, **kwargs: None
    except Exception:
        pass
    try:
        app = _get_app()
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    except OSError as e:
        busy = "Address already in use" in str(e) or getattr(e, "errno", None) == 98
        if busy:
            print(f"[web] ERROR: port {port} is already in use.")
            holders = _pids_listening_on_port(port)
            if holders:
                for pid in holders:
                    cmd = _cmd_for_pid(pid)
                    extra = f"  {cmd}" if cmd else ""
                    print(f"[web]   pid {pid}{extra}")
                print(f"[web] Stop it with: kill {' '.join(str(p) for p in holders)}")
            else:
                print(f"[web] Try: fuser -k {port}/tcp   or   ~/sub --port {port + 1}")
        else:
            print(f"[web] ERROR: {e}")
        raise
    except Exception as e:
        print(f"[web] ERROR: {e}")
        raise
