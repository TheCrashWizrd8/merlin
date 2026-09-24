# Quick reference — yolo-project
# Last updated: 24 September 2026

## Start stack (from anywhere)
~/sub                              # dashboard + ESP + cameras (YOLO off)
~/sub --yolo --backend hailo --timing   # YOLO at startup
~/sub --help                       # all flags

Install once: ln -sf ~/yolo-project/sub ~/sub

## Common flags
~/sub --no-camera                  # dashboard only
~/sub --serial-port /dev/ttyACM0
~/sub --no-xbox --no-gps
python scripts/test_telemetry.py   # simulated ESP data (UI dev)

## URLs
http://<pi-ip>:8080/                 # YOLO MJPEG stream (/video_feed)
http://<pi-ip>:8080/sub/             # sub dashboard (telemetry + control)

## Auto mode (YOLO)
Camera → controller (steer/tilt/drive + safety) → sub_motion.py:
  fins (gyro level) → aft steer → thruster → ballast (error_y height)
  linked flap: fins oppose aft-steer axis (Xbox + YOLO)
ESP: S2 … F … X …  and  B fore aft  via esp_bridge (single serial owner)
B button: emergency stop (halt YOLO + zero actuators)

## Camera
Expected: 640x480 MJPEG (FIT0819 endoscope). Startup logs fourcc=MJPG.
Slow FPS? Use --timing, img_size: 320 in model.yaml.
/sub/ dashboard: SSE live feed (/sub/api/stream) + MJPEG camera (/video_feed, ~15 FPS).

vlc v4l2:///dev/video0 --v4l2-chroma=MJPG
bash scripts/check_camera.sh

## ESP UART probe
python scripts/probe_esp_uart.py

## Docs
README.md          — project overview
STATUS.md          — current status (what's done / in progress)
docs/GUIDE.md      — full guide
docs/ESP32_SERIAL.md
config/hardware.yaml — approach: (safety), sub_motion: (auto actuators)
