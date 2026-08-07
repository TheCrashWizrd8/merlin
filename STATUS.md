# Project Status

_Last updated: August 2026_

---

## What This Project Does

An autonomous apple-detection RC submarine running on a Raspberry Pi 5. The Pi uses a USB endoscope camera to capture live video, runs a YOLOv8 model to detect apples in each frame, and outputs steering/thruster/tilt commands to guide the sub toward a target apple.

A parallel **sub vehicle stack** on the same Pi provides ESP32 telemetry (battery, depth, gyro, leaks, ballast), **layered sub actuator control** (fins → aft steer → thruster + ballast height trim), Xbox controller input, and a web dashboard at `/sub/`.

---

## Current Status

### Done

| Area | Detail |
|------|--------|
| **Camera** | DFRobot FIT0819 USB endoscope on `/dev/video0`, MJPEG 640×480 @ ~25 fps capture; fourcc logged at open |
| **Inference pipeline** | Camera → YOLO → tracker → controller → hardware / web stream |
| **Trained model** | `weights/best.pt` configured in `config/model.yaml` (YOLOv8n, 50 epochs) |
| **Dataset** | 697 labelled apple images (489 train / 178 val / 30 test), classes: `apple` + `damaged_apple` |
| **Controller** | Proportional + sustain-until-centred steering/tilt; size-based drive mapping |
| **Telemetry safety** | `src/telemetry_context.py` — leak stop, low-battery drive scale, alignment gating; **camera drives steer/tilt/drive** (not depth fusion) |
| **Unified sub serial** | `interface: sub` → `SubBridgeOutput` → `sub_state`; **single** `esp_bridge` owns USB/GPIO UART (no duplicate S/D/T serial client) |
| **Layered sub motion** | `src/sub_motion.py` — gyro fins level body → aft steer Y/Z → gated thruster → ballast fill/drain from vertical apple error |
| **Web stream** | `inference.py --web` — MJPEG at `/video_feed`; snapshot at `/snapshot` |
| **Inference perf** | Headless display when sub/web server active; `--timing` / `--quiet`; YOLO warmup; MJPEG fourcc warning |
| **ESP32 sub firmware** | `esp32/sub_rc/` — GPIO UART or USB to Pi, PCA9685 servos, L298N thruster, ballast, leak/ADC/IMU telemetry |
| **ESP bridge** | `src/esp_bridge.py` — reads `TEL` lines, sends `S2`/`B` at ~20 Hz; DTR/RTS fixes for USB ESP32 |
| **Sub dashboard** | `/sub/` UI — SSE live feed, MJPEG camera, ballast, actuators, diagnostics, serial monitor |
| **Dashboard live stream** | `GET /sub/api/stream` (SSE) — push updates on state change; REST routes kept for scripts |
| **Dashboard control persistence** | Manual default on bench; slider/mode state in `sessionStorage`; Xbox fallback when pad offline |
| **Xbox controller** | `src/xbox_controller.py` + `src/xbox_mapping.py` — gamepad → actuators when mode is `xbox`; bindings in `config/xbox_mapping.yaml` |
| **Pin reference** | `config/pins.yaml` — GPIO, PCA9685 channels, L298N, ballast, leak sensors |
| **Hardware config** | `config/hardware.yaml` — `interface: sub`, `sub_serial`, `approach:`, `sub_motion:` |
| **Colab notebook** | `train_colab.ipynb` for upload → train → download `best.pt` |
| **Utility scripts** | Camera check, UART probe, telemetry simulation |
| **Tests** | `tests/test_sub_motion.py` — fin/thruster gating and ballast height sign |

### In Progress

| Area | Detail |
|------|--------|
| **Field testing** | End-to-end YOLO auto + layered sub actuators on real hardware |
| **PCA9685 on sub** | `ENABLE_PCA9685` is `0` in firmware until I2C wiring is confirmed (GPIO 21/22) |
| **IMU / depth sensor** | Telemetry stubs present; gyro may read zeros until IMU wired — fins/thruster gating need real pitch/roll for full effect |

### Not Started / Future

| Area | Detail |
|------|--------|
| **Spektrum AR8020T receiver** | RC receiver decoding on ESP32 (manual mode currently uses web/Xbox) |
| **Multi-camera triangulation** | `cameras.num_cameras` placeholder in config |
| **Runtime pin config** | `config/pins.yaml` is reference-only; code still reads `hardware.yaml` and firmware defines |
| **Motion phase on dashboard** | `level` / `point` / `approach` computed in code; not yet shown as a UI badge |

---

## How to Run

### YOLO + sub stack (recommended — `interface: sub` in config)

```bash
cd ~/yolo-project
source .venv/bin/activate
python inference.py --web --timing
# YOLO stream: http://<pi-ip>:8080/
# Sub dashboard: http://<pi-ip>:8080/sub/
# Inference sets sub control mode to auto; ESP bridge starts automatically
```

With `interface: sub`, you do **not** need `--sub` — the sub dashboard and ESP bridge start automatically. Use `--no-sub` to disable.

### YOLO inference (headless, no web)

```bash
python inference.py --headless
```

### YOLO + web stream only (stub hardware)

Set `interface: stub` in `config/hardware.yaml`, then:

```bash
python inference.py --web
# Open http://<pi-ip>:8080
```

### Sub dashboard only (no YOLO)

```bash
python sub_server.py
# Open http://<pi-ip>:8080/sub/
```

### Simulated telemetry (UI dev, no ESP)

```bash
python scripts/test_telemetry.py
```

### Probe ESP32 on GPIO UART

```bash
python scripts/probe_esp_uart.py
```

---

## Key Settings

| File | Setting | Current Value |
|------|---------|---------------|
| `config/model.yaml` | `architecture` | `yolov8n` |
| `config/model.yaml` | `weights` | `weights/best.pt` |
| `config/model.yaml` | `img_size` | `640` (use `320` for more FPS) |
| `config/model.yaml` | `confidence` | `0.50` |
| `config/hardware.yaml` | `interface` | `sub` |
| `config/hardware.yaml` | `sub_serial.port` | `/dev/ttyACM0` (USB) or `/dev/serial0` (GPIO UART) |
| `config/hardware.yaml` | `approach.use_telemetry` | `true` (safety only) |
| `config/hardware.yaml` | `sub_motion.use_ballast_for_height` | `true` |
| `config/hardware.yaml` | `use_size_for_drive` | `true` |

---

## Architecture (high level)

```
USB Camera ──► YOLO ──► Tracker ──► Controller ──► ControlOutput
                         │              │
                         │              └── TelemetryContext (leak, battery, alignment)
                         │
                         ▼
              SubBridgeOutput (interface: sub)
                         │
         ┌───────────────┴────────────────┐
         │  sub_motion.plan_sub_motion   │
         │  fins → aft steer → thruster  │
         │  + ballast (error_y height)   │
         └───────────────┬────────────────┘
                         ▼
                   sub_state (auto actuators + ballast cmd)
                         │
                         ▼
              esp_bridge (single serial owner)
                         │
                         ▼
              /dev/ttyACM0 or /dev/serial0
              S2 / B / TEL
                         │
                         ▼
                    sub_rc.ino
              servos, thruster, ballast,
              leak, battery, depth, IMU
```

**Control split:** Camera drives **intent** (steer, tilt, drive, vertical apple error). Gyro drives **fins** (level body). Telemetry drives **safety only** (leak stop, battery scale) — not fixed depth targeting.

---

## Project Structure

```
yolo-project/
├── inference.py          # Main YOLO loop (+ --web, --sub, --timing, --quiet)
├── sub_server.py         # Sub dashboard without YOLO
├── train.py              # Training (GPU / Colab)
├── train_colab.ipynb     # Colab notebook
├── config/               # model, dataset, hardware, pins
├── src/
│   ├── sub_motion.py     # Layered auto: fins, steer, thruster, ballast
│   ├── telemetry_context.py
│   ├── hardware.py       # SubBridgeOutput
│   └── …                 # detector, tracker, controller, esp_bridge, sub_*
├── tests/
│   └── test_sub_motion.py
├── esp32/sub_rc/         # Primary ESP32 firmware
├── scripts/              # test_telemetry, probe_esp_uart, etc.
├── data/images/          # Training dataset
├── weights/best.pt       # Trained model
└── docs/                 # GUIDE, ESP32_SERIAL, etc.
```
