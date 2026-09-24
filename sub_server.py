#!/usr/bin/env python3
"""
sub_server.py
-------------
Same process as ``~/sub`` / ``run.py``: dashboard, cameras, ESP, GPS, Xbox.
YOLO starts from /sub/ or with --yolo.
"""

from src.app import main

if __name__ == "__main__":
    main()
