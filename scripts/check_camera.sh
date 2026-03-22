#!/bin/bash
# Camera troubleshooting for Raspberry Pi + USB camera (e.g. DFRobot FIT0819).
# Run from project root:  bash scripts/check_camera.sh

set -e
echo "=== 1. Video device nodes ==="
ls -la /dev/video* 2>/dev/null || echo "No /dev/video* devices found."

echo ""
echo "=== 2. User groups (need 'video' for camera access) ==="
groups
id

echo ""
echo "=== 3. V4L2 devices (v4l2-ctl) ==="
if command -v v4l2-ctl &>/dev/null; then
  v4l2-ctl --list-devices 2>/dev/null || echo "v4l2-ctl failed or no devices."
else
  echo "v4l2-ctl not installed. Install with: sudo apt install v4l-utils"
fi

echo ""
echo "=== 4. USB devices (look for your camera) ==="
lsusb

echo ""
echo "=== 5. Kernel messages (last 30 lines, USB/video related) ==="
dmesg | tail -30

echo ""
echo "=== 6. Processes using video devices ==="
fuser -v /dev/video* 2>/dev/null || echo "None or no devices."
