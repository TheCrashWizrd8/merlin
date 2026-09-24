#!/usr/bin/env python3
"""
run.py
------
Python launcher for the Pi sub stack (prefer ``~/sub`` from anywhere).

Starts the dashboard, stereo cameras (with reconnect), ESP, GPS, and Xbox.
YOLO stays off until you pick a backend on /sub/, or pass --yolo.

    ~/sub
    ~/sub --yolo --backend hailo --timing
    ~/sub --serial-port /dev/ttyACM0 --no-xbox

``sub_server.py`` and ``inference.py`` call the same ``main()``.
"""

from src.app import main

if __name__ == "__main__":
    main()
