#!/usr/bin/env python3
"""
inference.py
------------
Same process as ``~/sub`` / ``run.py``: dashboard, cameras, ESP, GPS, Xbox.
YOLO starts from /sub/ or with --yolo.

    ~/sub
    ~/sub --yolo --backend hailo --timing
"""

from src.app import main

if __name__ == "__main__":
    main()
