#!/usr/bin/env python3
"""
Quick test: run only the web server (no camera). Use to check if the website loads.
  python scripts/test_web_server.py
  Then open http://localhost:8080 (or http://<pi-ip>:8080 from another device)
  You should see "Waiting for camera..." until you stop with Ctrl+C.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.web_stream import _get_app, run_server

if __name__ == "__main__":
    print("Starting web server on http://0.0.0.0:8080")
    print("Open http://localhost:8080 in a browser.")
    try:
        import subprocess
        r = subprocess.run(["tailscale", "ip", "-4"], capture_output=True, text=True, timeout=2)
        if r.returncode == 0 and r.stdout and r.stdout.strip():
            print(f"Over Tailscale: http://{r.stdout.strip()}:8080")
    except Exception:
        pass
    print("Press Ctrl+C to stop.")
    _get_app()
    run_server(host="0.0.0.0", port=8080)
