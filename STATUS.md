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
| **Camera** | Dual FIT0819 USB endoscopes (stereo). Centres **16 cm** apart, **67°** FOV. Working size 640×480 (native size is resized if the two cameras differ) |
| **Inference pipeline** | Camera(s) → YOLO (NCNN) → tracker → optional stereo range → controller → hardware / web stream |
| **NCNN backend** | `backend: ncnn` in `config/model.yaml`; export via `scripts/export_model.py`. Pi 5: ~68 ms vs ~304 ms PyTorch |
| **Stereo range** | One shared YOLO model; left/right frames detected sequentially; `src/stereo.py` triangulates centres → HUD `Range` (metres) |
| **Trained model** | **`detect` only today:** `weights/best.pt` + `weights/best_ncnn_model/` (YOLOv8n, 50 epochs). Seg / other weights: add to catalog when ready |
| **YOLO model picker** | `/sub/` **Auto (YOLO)** → second button row switches catalog models live (`src/model_runtime.py`). API: `GET/POST /sub/api/models`. SSE stream includes `models` snapshot |
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
| **Tests** | `tests/test_sub_motion.py`, `tests/test_stereo.py`, `tests/test_model_runtime.py`, `tests/test_detector.py` |

### In Progress

| Area | Detail |
|------|--------|
| **Field testing** | End-to-end YOLO auto + layered sub actuators on real hardware |
| **I2C peripherals** | PCA9685 @ 0x40 + MPU6050 GY-521 @ 0x68 on shared SDA/SCL (GPIO 29/30) |

### Not Started / Future

| Area | Detail |
|------|--------|
| **Spektrum AR8020T receiver** | RC receiver decoding on ESP32 (manual mode currently uses web/Xbox) |
| **Stereo calibration** | Parallel-camera pinhole model (no undistort/rectify). Tune `fov_h_deg` / `focal_length_px` against a known distance |
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
# In Auto mode, use the YOLO model buttons to switch AI (see Key Settings → models)
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
| `config/model.yaml` | `task` | `detect` |
| `config/model.yaml` | `backend` | `ncnn` |
| `config/model.yaml` | `weights` | `weights/best.pt` (loads `best_ncnn_model/`) |
| `config/model.yaml` | `active_model` | `detect` (only deployed catalog entry today) |
| `config/model.yaml` | `models` | Catalog for dashboard picker — `seg` / `seg_fast` are placeholders until you add weights |
| `config/model.yaml` | `img_size` | `640` (use `320` only if you need more FPS; re-export NCNN) |
| `config/model.yaml` | `confidence` | `0.50` |
| `config/hardware.yaml` | `cameras.num_cameras` | `2` |
| `config/hardware.yaml` | `cameras.baseline_cm` | `16` |
| `config/hardware.yaml` | `cameras.fov_h_deg` | `67` |
| `config/hardware.yaml` | `interface` | `sub` |
| `config/hardware.yaml` | `sub_serial.port` | `/dev/ttyACM0` (USB) or `/dev/serial0` (GPIO UART) |
| `config/hardware.yaml` | `approach.use_telemetry` | `true` (safety only) |
| `config/hardware.yaml` | `sub_motion.use_ballast_for_height` | `true` |
| `config/hardware.yaml` | `use_size_for_drive` | `true` |

### YOLO model catalog (multiple AIs)

The Pi can run **several YOLO weights** and switch between them in **Auto** mode without a full restart. Catalog: **`config/model.yaml`** → `models:`.

| ID | Label | Weights | Task | Deployed |
|----|-------|---------|------|----------|
| `detect` | Detect | `weights/best.pt` | detect | **Yes** |
| `seg` | Seg (future) | `weights/best_seg.pt` | segment | No — add when trained |
| `seg_fast` | Seg 320 (future) | `weights/best_seg_320.pt` | segment | No |

**Use:** `/sub/` → **Auto (YOLO)** → **YOLO model** buttons. Or `POST /sub/api/models/select` with `{"id":"detect"}`. Implementation: `src/model_runtime.py` (hot reload via `Detector.reload()`).

**Per entry:** `label`, `weights`, `task`, `track_label`. With `backend: ncnn`, each `.pt` needs its own `weights/<stem>_ncnn_model/` export.

**Add later:** copy `.pt` → `python scripts/export_model.py --weights … --format ncnn` → add YAML entry → restart inference or pick from dashboard.

Full write-up: **`docs/GUIDE.md`** §11 and **`README.md`** § Multiple YOLO models.

---

## Architecture (high level)

```
USB Camera(s) ──► YOLO (one model, hot-swappable) ──► Tracker ──► Controller ──► ControlOutput
                         │                                ▲
                         └── stereo.triangulate (range_m) ─┘
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
│   ├── model_runtime.py  # YOLO catalog + hot-swap
│   ├── sub_motion.py     # Layered auto: fins, steer, thruster, ballast
│   ├── telemetry_context.py
│   ├── hardware.py       # SubBridgeOutput
│   └── …                 # detector, tracker, controller, esp_bridge, sub_*
├── tests/
│   └── test_sub_motion.py
├── esp32/sub_rc/         # Primary ESP32 firmware
├── scripts/              # test_telemetry, probe_esp_uart, etc.
├── data/images/          # Training dataset
├── weights/best.pt       # Detect model (catalog: detect)
├── weights/best_ncnn_model/
└── docs/                 # GUIDE, ESP32_SERIAL, etc.
```
