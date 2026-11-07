#!/usr/bin/env python3
"""Grab frames from a LinkBack USB camera, report FPS, and save a snapshot PNG.

    python scripts/linkback_snapshot.py                 # first camera
    python scripts/linkback_snapshot.py linkback:3-1    # camera on USB port 3-1
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import cv2

from src.camera import CameraError
from src.linkback_camera import LinkbackCamera, format_linkback_report


def main() -> int:
    spec = sys.argv[1] if len(sys.argv) > 1 else "linkback"
    out = Path(sys.argv[2]) if len(sys.argv) > 2 else ROOT / "linkback_snapshot.png"
    print(format_linkback_report())
    cam = LinkbackCamera(spec)
    try:
        cam.open()
    except CameraError as exc:
        print(f"open failed: {exc}")
        return 1
    try:
        frames = 0
        last = None
        t0 = time.monotonic()
        while time.monotonic() - t0 < 3.0:
            ok, frame = cam.read()
            if ok:
                frames += 1
                last = frame
        dt = time.monotonic() - t0
        print(f"{cam.device} port {cam.port_path}: {frames} frames in {dt:.1f}s "
              f"({frames / dt:.1f} fps), dropped {cam.frames_dropped}")
        if last is None:
            print("no frames received")
            return 1
        cv2.imwrite(str(out), last)
        print(f"saved {out} ({last.shape[1]}x{last.shape[0]})")
    finally:
        cam.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
