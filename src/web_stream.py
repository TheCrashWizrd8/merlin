"""
web_stream.py
-------------
Serves the live camera MJPEG feed over HTTP.
Run inference with --web to host; open http://<pi-ip>:8080/video_feed in a browser.
Sub dashboard routes are registered separately via register_sub_dashboard().
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np

# Lazy import Flask so the app can run without it until --web is used
_app = None
_frame_store: dict = {"bytes": None}
_frame_lock = threading.Lock()

# Placeholder JPEG sent before first real frame (avoids empty/broken stream in browser)
_PLACEHOLDER_JPEG: Optional[bytes] = None


def _get_placeholder_jpeg() -> bytes:
    """Single gray frame with 'Waiting for camera...' so /video_feed always returns valid MJPEG."""
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


def _get_app():
    global _app
    if _app is None:
        from flask import Flask, Response

        _app = Flask(__name__)

        def set_latest_frame_bgr(frame_bgr: np.ndarray) -> None:
            """Update the frame served to browsers. Call from inference loop."""
            if frame_bgr is None or frame_bgr.size == 0:
                return
            ok, buf = cv2.imencode(".jpg", frame_bgr)
            if ok:
                with _frame_lock:
                    _frame_store["bytes"] = buf.tobytes()

        def generate_feed():
            import time
            boundary = "frame"
            while True:
                with _frame_lock:
                    frame_bytes = _frame_store["bytes"]
                if not frame_bytes:
                    frame_bytes = _get_placeholder_jpeg()
                if frame_bytes:
                    yield (
                        b"--" + boundary.encode() + b"\r\n"
                        b"Content-Type: image/jpeg\r\n"
                        b"Content-Length: " + str(len(frame_bytes)).encode() + b"\r\n\r\n"
                        + frame_bytes + b"\r\n"
                    )
                time.sleep(0.066)

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
    img { max-width: 100%; border-radius: 8px; }
  </style>
</head>
<body>
  <h1>Apple RC Sub</h1>
  <p>Live camera feed. Sub dashboard: <a href="/sub/" style="color:#40916c">/sub/</a></p>
  <img src="/video_feed" alt="Live stream" />
</body>
</html>
"""

        @_app.route("/video_feed")
        def video_feed():
            return Response(
                generate_feed(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
                headers={
                    "Cache-Control": "no-cache, no-store, must-revalidate",
                    "Pragma": "no-cache",
                    "Expires": "0",
                },
            )

        @_app.route("/snapshot")
        def snapshot():
            with _frame_lock:
                frame_bytes = _frame_store["bytes"]
            if not frame_bytes:
                frame_bytes = _get_placeholder_jpeg()
            return Response(
                frame_bytes,
                mimetype="image/jpeg",
                headers={
                    "Cache-Control": "no-cache, no-store, must-revalidate",
                    "Pragma": "no-cache",
                    "Expires": "0",
                },
            )

        _app.set_latest_frame = set_latest_frame_bgr
    return _app


def set_latest_frame(frame_bgr: np.ndarray) -> None:
    """Update the frame shown on the web stream. Call from inference loop."""
    app = _get_app()
    if hasattr(app, "set_latest_frame"):
        app.set_latest_frame(frame_bgr)


def register_sub_dashboard(start_services: bool = True) -> None:
    """Register /sub/* routes on the Flask app (used by sub_server and inference --sub)."""
    from src.sub_web import register_sub_dashboard as _register
    app = _get_app()
    _register(app, start_services=start_services)


def run_server(host: str = "0.0.0.0", port: int = 5000) -> None:
    """Run the Flask server (blocking). Use in a daemon thread from inference."""
    try:
        app = _get_app()
        app.run(host=host, port=port, threaded=True, use_reloader=False)
    except OSError as e:
        if "Address already in use" in str(e) or "Errno 98" in str(e):
            print(f"[web] ERROR: Port {port} is already in use. Try: python inference.py --web --web-port 8080")
        else:
            print(f"[web] ERROR: {e}")
        raise
    except Exception as e:
        print(f"[web] ERROR: {e}")
        raise
