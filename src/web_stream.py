"""
web_stream.py
-------------
Serves the live camera feed and S/D/T control UI over HTTP.
Run inference with --web to host; open http://<pi-ip>:8080 in a browser.

Control source abstraction (control_source.py) allows manual web control now
and RC controller input later.
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
        from flask import Flask, Response, request, jsonify
        from src.control_source import set_mode, get_mode, set_manual, get_manual

        _app = Flask(__name__)
        _app.config["MAX_CONTENT_LENGTH"] = 64 * 1024  # 64KB for JSON

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
                time.sleep(0.1)

        @_app.route("/api/control", methods=["GET"])
        def api_control_get():
            m = get_manual()
            return jsonify({"mode": get_mode(), "s": m.s, "d": m.d, "t": m.t})

        @_app.route("/api/control", methods=["POST"])
        def api_control_post():
            data = request.get_json(silent=True) or {}
            if "mode" in data:
                set_mode(str(data["mode"]))
            m = get_manual()
            if "s" in data or "d" in data or "t" in data:
                s = float(data.get("s", m.s))
                d = float(data.get("d", m.d))
                t = float(data.get("t", m.t))
                set_manual(s, d, t)
                m = get_manual()
            return jsonify({"mode": get_mode(), "s": m.s, "d": m.d, "t": m.t})

        @_app.route("/")
        def index():
            return """
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Apple Car — Control</title>
  <style>
    * { box-sizing: border-box; }
    body {
      margin: 0;
      background: #1a1a1a;
      color: #e0e0e0;
      font-family: system-ui, -apple-system, sans-serif;
      min-height: 100vh;
      display: flex;
      flex-direction: column;
      align-items: center;
      padding: 1rem;
    }
    h1 { margin: 0 0 0.5rem; font-size: 1.25rem; font-weight: 600; }
    .sub { color: #888; font-size: 0.875rem; margin-bottom: 1rem; }
    .layout { display: flex; flex-wrap: wrap; gap: 1.5rem; justify-content: center; align-items: flex-start; max-width: 1200px; }
    .stream-wrap {
      background: #000;
      border-radius: 8px;
      overflow: hidden;
      box-shadow: 0 4px 20px rgba(0,0,0,0.4);
      flex: 0 1 640px;
    }
    .stream-wrap img { display: block; width: 100%; height: auto; }
    .controls {
      background: #252525;
      border-radius: 8px;
      padding: 1.25rem;
      min-width: 280px;
    }
    .controls h2 { margin: 0 0 1rem; font-size: 1rem; font-weight: 600; }
    .mode-row { display: flex; gap: 0.5rem; margin-bottom: 1rem; }
    .mode-btn {
      flex: 1;
      padding: 0.5rem;
      border: 1px solid #444;
      border-radius: 6px;
      background: #333;
      color: #e0e0e0;
      cursor: pointer;
      font-size: 0.9rem;
    }
    .mode-btn.active { background: #2d6a4f; border-color: #40916c; }
    .mode-btn:hover { background: #404040; }
    .ctrl-row { display: flex; align-items: center; gap: 0.75rem; margin-bottom: 0.75rem; }
    .ctrl-row label { width: 2rem; font-size: 0.9rem; font-weight: 500; }
    .ctrl-row input[type="range"] { flex: 1; accent-color: #40916c; }
    .ctrl-row .val { min-width: 3rem; font-size: 0.85rem; color: #aaa; }
  </style>
</head>
<body>
  <h1>Apple Car</h1>
  <p class="sub">Camera view and S/D/T control</p>
  <div class="layout">
    <div class="stream-wrap">
      <img src="/video_feed" alt="Live stream" />
    </div>
    <div class="controls">
      <h2>Control</h2>
      <div class="mode-row">
        <button class="mode-btn active" data-mode="auto">Auto</button>
        <button class="mode-btn" data-mode="manual">Manual</button>
      </div>
      <div class="ctrl-row">
        <label>S</label>
        <input type="range" id="s" min="-1" max="1" step="0.05" value="0">
        <span class="val" id="sVal">0.00</span>
      </div>
      <div class="ctrl-row">
        <label>D</label>
        <input type="range" id="d" min="-1" max="1" step="0.05" value="0">
        <span class="val" id="dVal">0.00</span>
      </div>
      <div class="ctrl-row">
        <label>T</label>
        <input type="range" id="t" min="-1" max="1" step="0.05" value="0">
        <span class="val" id="tVal">0.00</span>
      </div>
    </div>
  </div>
  <script>
    const sEl = document.getElementById('s');
    const dEl = document.getElementById('d');
    const tEl = document.getElementById('t');
    const sVal = document.getElementById('sVal');
    const dVal = document.getElementById('dVal');
    const tVal = document.getElementById('tVal');
    const btns = document.querySelectorAll('.mode-btn');

    function updateVal(el, valEl, v) {
      const n = Math.round(parseFloat(v) * 100) / 100;
      valEl.textContent = n.toFixed(2);
    }
    sEl.addEventListener('input', () => { updateVal(sEl, sVal, sEl.value); send(); });
    dEl.addEventListener('input', () => { updateVal(dEl, dVal, dEl.value); send(); });
    tEl.addEventListener('input', () => { updateVal(tEl, tVal, tEl.value); send(); });

    function send() {
      fetch('/api/control', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ mode: 'manual', s: +sEl.value, d: +dEl.value, t: +tEl.value })
      });
    }

    btns.forEach(b => {
      b.addEventListener('click', () => {
        const mode = b.dataset.mode;
        btns.forEach(x => x.classList.toggle('active', x === b));
        fetch('/api/control', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ mode }) })
          .then(r => r.json()).then(d => {
            sEl.value = d.s; dEl.value = d.d; tEl.value = d.t;
            updateVal(sEl, sVal, d.s); updateVal(dEl, dVal, d.d); updateVal(tEl, tVal, d.t);
          });
      });
    });

    fetch('/api/control').then(r => r.json()).then(d => {
      btns.forEach(b => b.classList.toggle('active', b.dataset.mode === d.mode));
      sEl.value = d.s; dEl.value = d.d; tEl.value = d.t;
      updateVal(sEl, sVal, d.s); updateVal(dEl, dVal, d.d); updateVal(tEl, tVal, d.t);
    });
  </script>
</body>
</html>
"""

        @_app.route("/video_feed")
        def video_feed():
            return Response(
                generate_feed(),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        _app.set_latest_frame = set_latest_frame_bgr
    return _app


def set_latest_frame(frame_bgr: np.ndarray) -> None:
    """Update the frame shown on the web stream. Call from inference loop."""
    app = _get_app()
    if hasattr(app, "set_latest_frame"):
        app.set_latest_frame(frame_bgr)


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
