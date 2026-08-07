# Quick reference — yolo-project

## Activate venv
cd ~/yolo-project && source .venv/bin/activate

## YOLO inference (interface: sub in config — recommended)
python inference.py --web --timing   # YOLO + ESP bridge + /sub/ dashboard
python inference.py --headless       # SSH / no browser
python inference.py --no-sub         # YOLO only, no sub stack
python inference.py --quiet          # skip per-frame terminal table (default with --web/sub)

## Sub dashboard only
python sub_server.py                 # no YOLO — ESP telemetry + actuators
python scripts/test_telemetry.py     # simulated ESP data (UI dev)

## URLs
http://<pi-ip>:8080/                 # YOLO MJPEG stream (/video_feed)
http://<pi-ip>:8080/sub/             # sub dashboard (telemetry + control)

## Auto mode (inference)
Camera → controller (steer/tilt/drive + safety) → sub_motion.py:
  fins (gyro level) → aft steer → thruster → ballast (error_y height)
ESP: S2 … F … X …  and  B fore aft  via esp_bridge (single serial owner)

## Camera
Expected: 640x480 MJPEG (FIT0819 endoscope). Startup logs fourcc=MJPG.
Slow FPS? Use --web (headless), --timing, img_size: 320 in model.yaml.
/sub/ dashboard: SSE live feed (/sub/api/stream) + MJPEG camera (/video_feed, ~15 FPS). No polling.

vlc v4l2:///dev/video0 --v4l2-chroma=MJPG
bash scripts/check_camera.sh

## ESP UART probe
python scripts/probe_esp_uart.py

## Docs
README.md          — project overview
STATUS.md          — current status (what's done / in progress)
docs/GUIDE.md      — full guide (§15 layered sub motion, §7 inference perf)
docs/ESP32_SERIAL.md
config/hardware.yaml — approach: (safety), sub_motion: (auto actuators)
