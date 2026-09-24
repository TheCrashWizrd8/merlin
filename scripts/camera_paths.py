#!/usr/bin/env python3
"""Print stable camera paths for hardware.yaml (plug-order safe)."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from src.camera import format_usb_camera_report, list_stable_usb_paths


def main() -> int:
    print(format_usb_camera_report())
    stable = list_stable_usb_paths()
    if not stable:
        print("\nNo USB by-path nodes found under /dev/v4l/by-path/")
        return 1
    print("\n# Paste into config/hardware.yaml (assign roles to physical USB ports):")
    for i, row in enumerate(stable, start=1):
        print(f"# camera_{i}: {row['video']}")
        print(f"#   {row['by_path']}")
    print("\n# Example:")
    print("cameras:")
    if len(stable) >= 3:
        print(f"  fov_device: {stable[0]['by_path']}")
        print(f"  left_device: {stable[1]['by_path']}")
        print(f"  right_device: {stable[2]['by_path']}")
    elif len(stable) == 2:
        print(f"  left_device: {stable[0]['by_path']}")
        print(f"  right_device: {stable[1]['by_path']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
