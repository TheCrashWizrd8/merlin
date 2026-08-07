# YOLO Apple RC Sub — Full System Guide

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Setup](#2-hardware-setup)
3. [Software Architecture](#3-software-architecture)
4. [Installation](#4-installation)
5. [Dataset & Training (Google Colab)](#5-dataset--training-google-colab)
6. [Training the Model](#6-training-the-model)
7. [Running Inference on the Pi](#7-running-inference-on-the-pi)
8. [ControlOutput & Control Behaviour](#8-controloutput--control-behaviour)
9. [Hardware Configuration (`hardware.yaml`)](#9-hardware-configuration-hardwareyaml)
10. [Swapping the Dataset](#10-swapping-the-dataset)
11. [Swapping the YOLO Model](#11-swapping-the-yolo-model)
12. [Hardware Output (`src/hardware.py`)](#12-hardware-output-srchardwarepy)
13. [Troubleshooting](#13-troubleshooting)
14. [Sub Vehicle System (`--sub`)](#14-sub-vehicle-system-sub)
15. [Layered Sub Motion (auto mode)](#15-layered-sub-motion-auto-mode)
16. [Sub Dashboard & API](#16-sub-dashboard--api)
17. [ESP32 Telemetry & Commands](#17-esp32-telemetry--commands)
18. [Complete Command Reference](#18-complete-command-reference)
19. [Scripts & Utilities](#19-scripts--utilities)
20. [Configuration Files Reference](#20-configuration-files-reference)
21. [ESP32 Firmware & Upload](#21-esp32-firmware--upload)
22. [Web Routes & HTTP API](#22-web-routes--http-api)
23. [Tests & Verification](#23-tests--verification)

### Quick command index

| Goal | Command |
|------|---------|
| YOLO + sub + web (production) | `python inference.py --web --timing` |
| Sub dashboard only | `python sub_server.py` |
| Train model (Colab/GPU) | `python train.py` |
| Fake dashboard data | `python scripts/test_telemetry.py` |
| Probe ESP serial | `python scripts/probe_esp_uart.py --port /dev/ttyACM0` |
| Test gamepad | `python -m src.xbox_controller` |
| Check camera | `bash scripts/check_camera.sh` |
| Flash ESP32 | `bash esp32/upload_from_pi.sh scan` then `bash esp32/upload_from_pi.sh` |
| Zip dataset for Colab | `bash scripts/zip_dataset.sh` |

---

## 1. System Overview

The system detects apples in a live USB camera feed using Ultralytics YOLO, tracks the best apple, and outputs normalised **steering**, **drive**, and **camera tilt** commands. The Raspberry Pi runs the full **sub vehicle stack** over **GPIO UART or USB** (`sub_rc`) with telemetry, ballast, and extra actuators.

```
USB Camera ──► Detector ──► Tracker ──► Controller ──► ControlOutput
                                              │              │
                                              │              └── TelemetryContext (safety)
                                              ▼
                                    stub / SubBridgeOutput
                                              │
                              sub_motion.plan_sub_motion (auto)
                              fins → aft steer → thruster + ballast
                                              │
                                              ▼
                                    esp_bridge → sub_rc.ino
                                    (S2 / B / TEL over serial)
```

### Goal behaviour (high level)

| Situation | Behaviour |
|-----------|-------------|
| Apple left / right of centre | Steering corrects (sustain-until-centred; see `hardware.yaml`) |
| Apple above / below centre | Camera tilt corrects; **ballast** fills/drains so sub rises/sinks toward apple height |
| Sub rolled or pitched (gyro) | **Fins** level body first; thruster gated until attitude improves |
| Apple far (small in frame) | Stronger forward thruster (when size-based drive is enabled) |
| Apple close (large in frame) | Weaker forward thruster |
| Leak detected (telemetry) | Drive forced to zero (safety) |
| Low battery (telemetry) | Drive scaled down (safety) |
| Lost / low confidence | Outputs decay to stop; brief hold of last valid target |

---

## 2. Hardware Setup

### Typical stack (Pi + ESP32 sub)

| Component | Role |
|-----------|------|
| Raspberry Pi 5 | Main compute (YOLO + control loop + web dashboard) |
| USB endoscope camera | Live YOLO input (`/dev/video0`) |
| ESP32-S3 (USB or GPIO UART to Pi) | Sub actuators, ballast, leak/ADC sensors, telemetry |
| PCA9685 (I2C on ESP32) | Aft steer Y/Z + fore fin servos (when enabled) |
| L298N + DC motor | Thruster |
| Fore/aft ballast | DC motors + linear pots (level feedback) |

**Pi ↔ ESP32 wiring:** Two options — pick one and set `sub_serial.port` in `config/hardware.yaml`.

| Connection | Pi side | ESP32 (`sub_rc`) | Device |
|------------|---------|------------------|--------|
| **USB (default)** | USB cable to ESP32-S3 | USB CDC | `/dev/ttyACM0` (or `/dev/serial/by-id/...`) |
| **GPIO UART** | GPIO14 TX (pin 8) → ESP RX (GPIO 44) | GPIO15 RX (pin 10) ← ESP TX (GPIO 43) | `/dev/serial0` |

Current default in `config/hardware.yaml`: **`sub_serial.port: "/dev/ttyACM0"`**. The ESP bridge also auto-detects Espressif devices under `/dev/serial/by-id/` when present.

Firmware and pin map: **`esp32/README.md`**, **`config/pins.yaml`**, **§17** and **§21**.

### Hardware interface (`config/hardware.yaml`)

| `interface` | Use case |
|-------------|----------|
| `stub` | No hardware; prints commands only (development) |
| `sub` | Route YOLO output to sub actuators via `esp_bridge` (production) |

---

## 3. Software Architecture

### Module responsibilities

| File | Responsibility |
|------|----------------|
| `src/detector.py` | Ultralytics YOLO; returns `Detection` with bbox + class |
| `src/tracker.py` | Picks target apple; bbox centre; `error_x` / `error_y`; bbox size |
| `src/controller.py` | `TrackResult` → `ControlOutput`; smoothing; size-based drive; telemetry **safety** only |
| `src/telemetry_context.py` | Read-only ESP snapshot (depth, battery, gyro, leak, ballast) for controller + sub motion |
| `src/hardware.py` | Maps `ControlOutput` to stub or `SubBridgeOutput` (single serial path via esp_bridge) |
| `src/camera.py` | USB camera via OpenCV; MJPEG preferred; fourcc logged at open |
| `src/display.py` | Bbox, crosshair, HUD (FPS, errors, proximity), gauges |
| `src/control_source.py` | Manual vs auto for YOLO S/D/T (used by sub auto mode) |
| `src/web_stream.py` | Flask MJPEG feed + `/snapshot` when using `--web` or sub stack |
| `src/dataset.py` | Resolves `data.yaml` (local or optional Roboflow download) |
| `src/esp_bridge.py` | Serial to ESP32: telemetry in, S2/B commands out (~20 Hz) |
| `src/sub_state.py` | Thread-safe sub telemetry, control modes, serial log |
| `src/sub_control.py` | Entry point for `yolo_to_sub_motion()` → `sub_motion.py` |
| `src/sub_motion.py` | Layered auto planner: fins, aft steer, thruster gating, ballast height |
| `src/sub_web.py` | `/sub/*` Flask routes and dashboard |
| `src/xbox_controller.py` | Gamepad reader (pygame thread, hot-plug) |
| `src/xbox_mapping.py` | Loads `config/xbox_mapping.yaml` → actuators + ballast |
| `src/xbox_stick_filter.py` | EMA + hold filter for Bluetooth stick jitter |
| `src/serial_util.py` | Serial port open helpers (USB boot grace, by-id) |
| `sub_server.py` | Standalone sub dashboard (no YOLO) |
| `train.py` | Training on GPU/Colab (not on Pi) |
| `train_colab.ipynb` | Colab notebook — upload dataset, train, download weights |
| `inference.py` | Main YOLO loop on Pi (+ `--web`, `--sub`) |

### Data flow per frame

```
cam.read()
    └──► detector.detect(frame)     → List[Detection]
              └──► tracker.update(...)  → TrackResult
                        └──► TelemetryContext.from_sub_state()  (when interface: sub)
                        └──► controller.compute(track, telemetry)  → ControlOutput
                                  ├──► SubBridgeOutput.apply(output)
                                  │         └──► sub_motion.plan_sub_motion(...)
                                  │                   → actuators + ballast → sub_state
                                  ├──► output.pretty()  → terminal (unless --quiet)
                                  └──► display.draw(frame, output) → web / HUD
```

When `interface: sub`, **one** serial client (`esp_bridge`) sends `S2` and `B` from `sub_state`. There is no separate legacy S/D/T serial output competing on the same port.

### Normalisation

All actuator fields in `ControlOutput` are **-1.0 … +1.0**. `hardware.yaml` maps them to PWM ranges per channel.

---

## 4. Installation

### On the Pi 5 (inference)

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y libopencv-dev python3-opencv

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

> **Note:** On ARM64 Pi, if `torch` fails, install CPU wheels, e.g.  
> `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`

### On a training machine (GPU / Colab)

```bash
pip install -r requirements.txt
```

For **Google Colab**, use a **GPU runtime** (Runtime → Change runtime type → GPU).

---

## 5. Dataset & Training (Google Colab)

Training data must be a **YOLO-format** tree with **`data.yaml`** at the root (train/val images and labels). The recommended path is **[Google Colab](https://colab.research.google.com/)** so you don’t need a local GPU.

### 5.1 Get the project into Colab

- **Option A:** Upload a zip of this repo, unzip, `cd` into the folder.
- **Option B:** Clone from Git (`git clone ...`) if the repo is hosted.

### 5.2 Get the dataset into Colab

Pick one:

- **Upload a zip:** Upload the dataset zip, unzip, e.g. `!unzip -q apple-dataset.zip -d data/apple-yolov8`
- **Google Drive:**  
  `from google.colab import drive`  
  `drive.mount('/content/drive')`  
  Then set `local_path` to the folder under Drive that contains `data.yaml` (e.g. `"/content/drive/MyDrive/datasets/apple-yolov8"` — use absolute paths in YAML if needed).

### 5.3 Point `config/dataset.yaml` at the data

```yaml
source: local
local_path: "data/apple-yolov8"   # or your unzipped / Drive path
```

`local_path` must resolve to a directory (or `data.yaml` file) that Ultralytics can use.

### 5.4 Install dependencies (Colab cell)

```python
!pip install -q ultralytics pyyaml roboflow python-dotenv
```

(Optional: `!pip install -r requirements.txt` if you uploaded the full repo.)

### 5.5 Run training (Colab cell)

```python
%cd /content/yolo-project   # adjust to your project path
!python train.py
```

Weights are written to **`weights/best.pt`**. Download from the **Files** sidebar (left) or:

```python
from google.colab import files
files.download('weights/best.pt')
```

### 5.6 Optional: Roboflow inside Colab

`config/dataset.yaml` can use `source: roboflow` with `ROBOFLOW_API_KEY` set in the environment or a Colab secret — then `train.py` will download the dataset when you run it. See `src/dataset.py`.

---

## 6. Training the Model

Run on a **GPU machine or Google Colab**, not on the Pi.

```bash
python train.py
```

This reads `config/model.yaml` and `config/dataset.yaml`, resolves `data.yaml`, trains (default: epochs/batch from `model.yaml`), and writes **`weights/best.pt`**.

### Common options

| Flag | Overrides | Default (from `config/model.yaml`) | Description |
|------|-----------|-----------------------------------|-------------|
| `--epochs N` | `epochs` | `50` | Training epochs |
| `--batch N` | `batch_size` | `16` | Batch size (reduce to `8` on low VRAM) |
| `--model NAME` | `architecture` | `yolov8n` | Model string, e.g. `yolov8s`, `yolo11n` |
| `--imgsz N` | `img_size` | `640` | Square input size in pixels |
| `--resume` | — | off | Resume from `weights:` path in `model.yaml` |
| `--device DEV` | — | auto | e.g. `cpu`, `0`, `0,1`, `mps` |

Examples:

```bash
python train.py
python train.py --model yolov8s --epochs 100 --batch 8
python train.py --resume --device 0
python train.py --device cpu   # no GPU
```

Output: **`weights/best.pt`** plus Ultralytics run logs under **`runs/`**.

### After training

1. Copy **`weights/best.pt`** to the Pi (USB, `scp`, or download from Colab).
2. Update `config/model.yaml`: `weights: weights/best.pt`
3. Run `python inference.py` on the Pi.

---

## 7. Running Inference on the Pi

### Recommended (production — `interface: sub`)

With `interface: sub` in `config/hardware.yaml`, inference **automatically** starts the ESP bridge, sub dashboard, and sets control mode to **auto**:

```bash
python inference.py --web --timing
```

| URL | Content |
|-----|---------|
| `http://<pi-ip>:8080/` | YOLO annotated MJPEG (`/video_feed`) |
| `http://<pi-ip>:8080/sub/` | Sub diagnostics dashboard |

Use `--no-sub` to run YOLO without the sub stack. Use explicit `--sub` to force sub stack when `interface: stub`.

OpenCV **display window is skipped** whenever the web/sub server is active (avoids `imshow` killing FPS over VNC/SSH).

### With local display window

Set `interface: stub` and run without `--web`:

```bash
python inference.py
```

Press **q** to quit.

### Headless (SSH, no browser)

```bash
python inference.py --headless
```

### Web UI only (`--web`, stub hardware)

```bash
python inference.py --web
```

Open `http://<pi-ip>:8080` (or `--web-port`). Live MJPEG at `/video_feed`.

### Inference flags

| Flag | Default | Description |
|------|---------|-------------|
| `--device N` | `0` | USB camera index |
| `--width` / `--height` | `640` / `480` | Capture size |
| `--headless` | off | No OpenCV window (also auto-on when sub/web server runs) |
| `--print-on-detect` | off | Print ControlOutput table only when apple detected |
| `--quiet` | off when sub/web active | Skip per-frame ControlOutput table in terminal |
| `--timing` | off | Log `cap` / `yolo` / `draw` ms and HUD FPS every 30 frames |
| `--tracker-strategy` | `best_confidence` | `best_confidence` or `closest_to_centre` |
| `--apple-label` | `apple` | Class label to track |
| `--web` | off | Flask server + browser stream at `/` |
| `--web-host` / `--web-port` | `0.0.0.0` / `8080` | Bind address and port |
| `--serial-port` | config | Override e.g. `/dev/ttyACM1` |
| `--control-profile` | `config` | `config`, `stable`, or `aggressive` — see **§8.1** |
| `--sub` | auto when `interface: sub` | Enable sub dashboard + ESP bridge |
| `--no-sub` | off | Disable sub stack even when `interface: sub` |
| `--no-xbox` | off | Skip Xbox controller polling |
| `--tolerate-missing-devices` | off | Keep running if camera or serial missing (manual-only fallback) |

### Performance notes

Expected on Pi 5 with yolov8n @ 640×480: roughly **8–15 FPS** end-to-end (HUD `FPS:` line). The FIT0819 endoscope captures MJPEG at ~25 FPS; YOLO is the main CPU load.

| Symptom | Likely cause |
|---------|--------------|
| ~1 FPS in terminal/HUD | OpenCV window over VNC — use `--web` / sub mode (headless) or `--headless` |
| Slow `/sub/` camera vs HUD | Camera capture/YOLO is the bottleneck — check `--timing`; dashboard MJPEG matches `/video_feed` (~15 FPS server-side) |
| Slow `yolo=` in `--timing` | Lower `img_size` to `320` in `config/model.yaml` |
| Slow `cap=` in `--timing` | Camera not in MJPEG — check startup line `fourcc=MJPG` |

Example `--timing` output:

```
[perf] cap=15ms yolo=70ms ctrl+draw=8ms total=93ms hud_fps=10.8
```

---

## 8. ControlOutput & Control Behaviour

### Fields (see `src/controller.py`)

| Field | Description |
|-------|-------------|
| `steering_servo`, `drive_motor`, `camera_tilt_servo` | Normalised -1…+1 |
| `apple_detected` | Whether a track is active |
| `target_x`, `target_y` | Pixel centre of apple |
| `bbox_x1`…`bbox_y2` | Tracked detection bounding box |
| `bbox_width`, `bbox_height`, `bbox_area`, `frame_area` | Size proxy for distance |
| `size_ratio_raw`, `size_ratio_filtered` | `bbox_area/frame_area` (instant / smoothed) |
| `error_x`, `error_y` | Normalised offset from frame centre |
| `confidence` | Detection confidence |
| `chosen_label` | Class name (e.g. `apple`) |
| `proximity_t` | 0 = far, 1 = close (from bbox size) |
| `approach_note` | Human-readable safety/alignment notes |
| `depth_m_used` | Display-only depth from telemetry (not used to steer) |
| `timestamp` | `time.time()` |

### Behaviour (summary)

- **Smoothing** on errors (`smoothing_alpha` in `hardware.yaml`).
- **Sustain-until-centred** steering/tilt: constant command until error inside deadzone (not a gentle proportional ramp).
- **Size-based drive** (optional): `use_size_for_drive` maps apparent apple size to **D** — typically **far** (small bbox) → higher `size_drive_far`, **close** (large bbox) → lower `size_drive_close`. Tune `size_min_ratio` / `size_max_ratio` to match HUD **Size** values.

### Telemetry safety (`approach:` in `hardware.yaml`)

When `interface: sub`, `TelemetryContext.from_sub_state()` is passed to `controller.compute()`. Telemetry affects drive **only** for safety and alignment — **not** fixed depth targeting:

| Condition | Effect |
|-----------|--------|
| Leak triggered (`stop_on_leak: true`) | `drive_motor = 0` |
| Battery below `min_battery_v` | Drive scaled by `low_battery_drive_scale` |
| Apple off-centre (`max_error_x/y_for_drive`) | Forward drive reduced until aligned |

Steering, tilt, and drive **intent** always come from the camera. Depth from ESP is shown on the HUD for display only.

### 8.1 Control profiles (`--control-profile`)

Profiles override selected keys in `config/hardware.yaml` at runtime (base values still load from YAML first).

| Profile | Effect |
|---------|--------|
| **`config`** | Use `hardware.yaml` as-is (default) |
| **`stable`** | More smoothing/damping, stricter confidence (`0.50`), longer hold on dropouts, lower drive while turning |
| **`aggressive`** | Less smoothing, lower confidence threshold (`0.35`), faster response, higher drive while turning |

Example:

```bash
python inference.py --web --control-profile stable
```

Overridden fields are defined in `src/controller.py` → `CONTROL_PROFILES`.

### Example code

```python
from src.detector import Detector
from src.tracker import Tracker
from src.controller import Controller

detector   = Detector()
tracker    = Tracker()
controller = Controller.from_hardware_config()

# ... frame from Camera ...
detections = detector.detect(frame)
track      = tracker.update(detections, frame.shape, apple_label="apple")
output     = controller.compute(track)

print(output.to_dict())
print(output.pretty())
```

---

## 9. Hardware Configuration (`hardware.yaml`)

Key sections:

- **`interface`**: `stub` | `sub`
- **`sub_serial`**: `port`, `baud_rate` (ESP32 link)
- **`cameras.num_cameras`**: 1–3 (placeholder for future multi-camera)
- **Control**: `deadzone`, `min_steer_command`, `min_tilt_command`, `min_drive_command`, `smoothing_alpha`, `confidence`, `hold_missed_frames`, etc.
- **Size-based drive**: `use_size_for_drive`, `size_drive_far`, `size_drive_close`, `size_min_ratio`, `size_max_ratio`, `size_smoothing_alpha`, `size_curve` (`sqrt` or `linear`)
- **`approach:`**: telemetry safety — `use_telemetry`, `stop_on_leak`, `min_battery_v`, `low_battery_drive_scale`, `max_error_x/y_for_drive`
- **`sub_motion:`**: layered auto actuators — fin gains, attitude limits, ballast height trim (`ballast_height_gain`, `ballast_error_deadzone`, etc.)

See inline comments in `config/hardware.yaml` for tuning.

---

## 10. Swapping the Dataset

1. Place a YOLO dataset so it contains `data.yaml` (e.g. unzip on **Colab** or copy to your machine).
2. Set in `config/dataset.yaml`:

```yaml
source: local
local_path: "data/your-dataset-folder"
```

3. Run `python train.py` (e.g. in Colab) or point `weights` in `model.yaml` to existing `.pt` files.

---

## 11. Swapping the YOLO Model

Edit `config/model.yaml`:

```yaml
architecture: yolov8s
weights: weights/best.pt
```

Approximate Pi5 performance (CPU inference, varies by resolution):

| Model | Pi5 inference (indicative) |
|-------|----------------------------|
| yolov8n | Fastest |
| yolov8s | Slower |
| yolov8m | Much slower |

Use `img_size: 320` or `640` per `model.yaml` to trade accuracy vs speed.

---

## 12. Hardware Output (`src/hardware.py`)

The project implements hardware output via `from_config()`:

```python
from src.hardware import from_config as hardware_from_config

hardware = hardware_from_config()
hardware.apply(output)
# ...
hardware.close()
```

`inference.py` does this automatically. With `interface: sub`, **`SubBridgeOutput`**:

1. Reads live telemetry via `TelemetryContext.from_sub_state()`
2. Calls `sub_motion.plan_sub_motion(ControlOutput, telemetry)` when sub control mode is **auto**
3. Writes actuators to `sub_state.set_auto_actuators()` and ballast to `set_ballast_commands()`
4. Lets **`esp_bridge`** send `S2` / `B` — the only serial writer on the port

There is no second serial client sending legacy S/D/T lines. Manual and Xbox modes use the dashboard or gamepad; auto mode is driven by YOLO each frame.

---

## 13. Troubleshooting

### Camera not found

```
CameraError: Cannot open camera device '0'
```

```bash
ls /dev/video*
python inference.py --device 1
```

### Weights missing

Train or set `weights:` in `config/model.yaml`, or use empty `weights` to download pretrained backbone (development only).

### Colab: dataset path wrong / `data.yaml` not found

- Confirm `local_path` matches the folder that **contains** `data.yaml` (or is the path to `data.yaml`).
- After unzipping, list files: `!ls -la data/your-folder/` and open `data.yaml` paths if needed.
- **Drive paths:** use the full path from `drive.mount()` (e.g. `/content/drive/MyDrive/...`).

### Roboflow (optional `source: roboflow`)

```bash
export ROBOFLOW_API_KEY="rf_xxxx"
pip install roboflow
```

### Serial / ESP32

- `ls /dev/ttyACM* /dev/ttyUSB*`
- Override: `python inference.py --serial-port /dev/ttyACM1`
- See **`docs/ESP32_SERIAL.md`** and **`docs/PI_ESP32_COMPATIBILITY.md`**

### Slow inference on Pi

- Use `yolov8n`, lower `img_size` to `320` in `config/model.yaml`.
- Run with `--web` or `interface: sub` so OpenCV window is skipped (headless).
- Use `--timing` to see whether `cap` or `yolo` dominates.
- Confirm camera startup shows `fourcc=MJPG`. If not:

```bash
v4l2-ctl -d /dev/video0 --set-fmt-video=width=640,height=480,pixelformat=MJPG
```

- Judge FPS from HUD `FPS:` or `[perf]` lines — dashboard camera uses the same MJPEG feed as `/video_feed`.

### Camera not connected

Without a USB camera, inference exits on open failure unless `--tolerate-missing-devices`. The web dashboard shows a gray **“Waiting for camera…”** placeholder (400×120) at ~5 FPS — not a real feed.

### Display over SSH

Use `--headless`, `--web`, or `interface: sub` (auto-headless when server runs). Avoid `cv2.imshow` over X forwarding — it can drop the loop to ~1 FPS.

---

## 14. Sub Vehicle System (`--sub`)

The sub stack uses **USB serial or GPIO UART** (see **`sub_serial`** in `config/hardware.yaml`) and the **`esp32/sub_rc/`** firmware.

### Entry points

| Command | YOLO | Sub dashboard | ESP bridge |
|---------|------|---------------|------------|
| `python sub_server.py` | No | Yes (`/sub/`) | Yes |
| `python inference.py` (with `interface: sub`) | Yes | Yes (auto) | Yes (auto) |
| `python inference.py --web` | Yes + `/` stream | Yes if `interface: sub` | Yes if `interface: sub` |
| `python inference.py --no-sub` | Yes | No | No |
| `python scripts/test_telemetry.py` | No | Yes (simulated) | No |
| `python scripts/simulate_esp_serial.py` | No | With `sub_server --serial-port …` | Simulated ESP |

**`sub_server.py` flags:** `--host`, `--port`, `--serial-port`, `--no-camera`, `--camera-device`, `--no-xbox`, `--no-esp` — see **§18.2**.

When inference starts with the sub stack, it sets **`get_sub_state().set_control_mode("auto")`** so YOLO drives actuators immediately.

### Control modes

The sub dashboard uses three modes (stored in `src/sub_state.py`):

| Mode | Source | When to use |
|------|--------|-------------|
| `manual` | Web UI or `POST /sub/api/control` | **Default on bench** — sliders stay put until you move them |
| `xbox` | Xbox/gamepad sticks and triggers | Manual driving when a pad is connected |
| `auto` | `sub_motion.plan_sub_motion()` | When inference is running (YOLO + layered actuators) |

**Inference auto mode:** On startup, inference sets server mode to **auto**. The dashboard may restore **manual** from browser `sessionStorage` on first load — click **Auto (YOLO)** on the dashboard if actuators stay at zero.

**Xbox fallback:** If mode is `xbox` but no controller is connected, the Pi keeps sending your **manual slider values** (not stale stick input).

**Dashboard persistence:** Mode, manual actuator sliders, and ballast sliders are saved in **`sessionStorage`**. Settings survive normal tab refresh.

See **[§15 Layered Sub Motion](#15-layered-sub-motion-auto-mode)** for how auto mode maps camera + gyro to actuators and ballast.

### Connecting the Xbox controller (Bluetooth)

Pairing is done **once at the OS level**. The project does not handle Bluetooth itself — after Linux exposes the pad as a gamepad, `sub_server.py` or `inference.py --sub` auto-detects it.

#### Supported controllers

| Controller | Bluetooth on Pi |
|------------|-----------------|
| Xbox Series X \| S | Yes |
| Xbox One S / Xbox One X (Share button, no seam around Xbox button) | Yes |
| Original Xbox One (no Share button) | No — USB or Microsoft wireless adapter only |
| Xbox 360 wireless | No — USB or Microsoft wireless adapter only |

#### Pair on the Pi (one-time)

1. Put the controller in pairing mode: hold the **Sync** button (small button near the USB port) until the Xbox logo **flashes rapidly**.
2. On the Pi, pair via `bluetoothctl` or the desktop Bluetooth menu.

**`bluetoothctl` (headless or SSH):**

```bash
bluetoothctl
power on
agent on
default-agent
scan on
# Wait for "Xbox Wireless Controller" and note the MAC address (XX:XX:XX:XX:XX:XX)
scan off
pair XX:XX:XX:XX:XX:XX
trust XX:XX:XX:XX:XX:XX
connect XX:XX:XX:XX:XX:XX
quit
```

The Xbox light should go **solid** (not flashing). You should see `/dev/input/js0` appear.

**Desktop UI:** Bluetooth tray icon → Add Device → **Xbox Wireless Controller** → Pair.

#### Verify before starting the sub stack

```bash
cd ~/yolo-project
source .venv/bin/activate
python -m src.xbox_controller
```

Lists detected pads and prints live stick/trigger values. Ctrl+C to quit.

#### Run the sub stack

```bash
python sub_server.py
# or
python inference.py --sub
```

Open **http://\<pi-ip\>:8080/sub/** — the status badge shows **Xbox — connected** when a pad is present. Default control mode is **manual** (use the **Manual** tab or move a slider; switch to **Xbox** when you want stick control).

USB also works if you prefer a cable; no separate pairing step is needed.

#### Troubleshooting

| Symptom | Fix |
|---------|-----|
| Pairing fails or drops immediately | Update controller firmware on Windows/Xbox via the **Xbox Accessories** app, then pair again |
| Pi shows connected but controller keeps flashing | Same — firmware update is the most common fix |
| Paired but no stick input | Disable ERTM (see below); for Series X \| S, try the [xpadneo](https://github.com/atar-axis/xpadneo) driver |
| Permission denied on `/dev/input/*` | `sudo usermod -aG input $USER` then log out and back in |

**Disable ERTM** (helps many Xbox + Raspberry Pi Bluetooth issues):

```bash
echo 'options bluetooth disable_ertm=Y' | sudo tee /etc/modprobe.d/bluetooth-xbox.conf
sudo reboot
```

Then pair again from scratch.

### Xbox mapping

Bindings are defined in **`config/xbox_mapping.yaml`** (path set by `xbox.mapping_file` in `config/hardware.yaml`). Edit that file to change controls without touching Python.

Implementation: **`src/xbox_mapping.py`** reads the YAML and maps raw pad state → `SubActuators` + fore/aft ballast.

#### Current layout (see YAML for live values)

| Input | Action |
|-------|--------|
| Left stick | Aft steer Y/Z (dual-axis stern servos) |
| Right stick | Fore fins (polar — up/down both, diagonals single fin) |
| **RB** | Thruster forward |
| **RT** | Thruster reverse |
| **LB (L1)** | Drain ballast (both tanks) |
| **LT (L2)** | Fill ballast (both tanks, analog) |
| **D-pad up + L1/L2** | Fore tank drain/fill |
| **D-pad down + L1/L2** | Aft tank drain/fill |

Right-stick fin angles use **snap zones** (`snap_degrees` in `config/xbox_mapping.yaml`, default ±5°):

| Bearing (from up) | Fins |
|-------------------|------|
| 0° ± snap | Both up |
| ±45° ± snap | Single fin up (left or right) |
| ±90° ± snap | Both neutral |
| ±135° ± snap | Single fin down |
| 180° ± snap | Both down (equal) |
| Between zones | Normal blend between nearest keypoints |

Verify live mapping:

```bash
python -m src.xbox_controller
```

Shows raw sticks, mapped actuators, and ballast commands. Ctrl+C to quit.

#### Stick drift

Sticks use a **circular deadzone** (total deflection, not per-axis) so small diagonal drift at rest maps to zero.

Optional **EMA smoothing** ignores brief Bluetooth dropouts so values do not snap to zero while the stick is held:

```yaml
xbox:
  deadzone: 0.18          # raise to 0.20–0.25 if actuators still twitch at rest
  trigger_deadzone: 0.08  # ignore LT/RT below this (ghost fill)
  smoothing_alpha: 0.35   # 0 = off; 0.3–0.5 typical
  stick_hold_ms: 120       # hold last good read through short zero glitches
  release_alpha: 0.55      # decay when stick returns to centre (not instant snap)
```

Set `smoothing_alpha: 0` if you want raw response with deadzone only.

Optional **per-stick** override in **`config/xbox_mapping.yaml`**:

```yaml
sticks:
  left:
    deadzone: 0.22
  right:
    deadzone: 0.22
```

Requires `pygame`. Skip with `--no-xbox` or set `xbox.enabled: false` in config.

### Xbox / sub config

In **`config/hardware.yaml`**:

```yaml
sub_serial:
  port: "/dev/ttyACM0"
  baud_rate: 115200

xbox:
  enabled: true
  deadzone: 0.18
  trigger_deadzone: 0.08
  poll_hz: 30
  scan_interval_s: 2.0
  mapping_file: config/xbox_mapping.yaml
  # device_index: 0   # omit to auto-pick (prefers Xbox-named devices)
```

Example **`config/xbox_mapping.yaml`** structure:

```yaml
sticks:
  left:
    aft_steer_y: x
    aft_steer_z: y
    invert_y: true
  right:
    mode: polar_fins

ballast:
  drain_button: lb
  fill_trigger: lt
  dpad_up_tank: fore
  dpad_down_tank: aft
  default_tank: both

thruster:
  forward_button: rb
  reverse_button: rt
```

Pin reference: **`config/pins.yaml`**.

---

## 15. Layered Sub Motion (auto mode)

Implemented in **`src/sub_motion.py`**, called from **`SubBridgeOutput`** via **`yolo_to_sub_motion()`**.

### Design principle

| Layer | Source | Purpose |
|-------|--------|---------|
| **Camera** | YOLO tracker `error_x`, `error_y`, size | Steer/tilt/drive **intent** |
| **Gyro** | ESP `TEL gyro pitch roll yaw` | Fin leveling, thruster/steer gating |
| **Telemetry** | Leak, battery | Safety only (in `controller.py`) |
| **Ballast** | Vertical `error_y` | Rise/sink toward apple height |

There is **no** fixed `target_depth_m` — the apple can be at any height; ballast follows vertical error in the camera frame.

### Control order (each frame)

1. **Fins (`finLeft`, `finRight`)** — proportional roll/pitch correction from gyro toward level body.
2. **Aft steer Y/Z** — camera horizontal/vertical errors (`steering_servo`, `camera_tilt_servo`), scaled down while still tilted.
3. **Thruster (`thrusterX`)** — forward drive from controller, gated by attitude + alignment.
4. **Ballast (fore + aft)** — fill/drain when apple is below/above frame centre.

### Tracker convention

| Field | Meaning |
|-------|---------|
| `error_x` | −1 = apple at left edge … +1 = right edge |
| `error_y` | −1 = apple **above** centre … +1 = **below** centre |

### Ballast height trim

| Apple position | `error_y` | Ballast command |
|----------------|-----------|-----------------|
| Below centre (sub should sink) | > 0 | **Fill** (+) both tanks |
| Above centre (sub should rise) | < 0 | **Drain** (−) both tanks |
| Near centre (deadzone) | ≈ 0 | Stop (0) |

Tunable in `config/hardware.yaml` under **`sub_motion:`**:

```yaml
sub_motion:
  level_roll_gain: 0.035
  level_pitch_gain: 0.025
  max_roll_deg: 12.0
  max_pitch_deg: 10.0
  require_level_for_steer: true
  require_level_for_drive: true
  use_ballast_for_height: true
  ballast_height_gain: 0.55
  ballast_error_deadzone: 0.12
```

### Motion phases (internal)

| Phase | When | What you see |
|-------|------|--------------|
| `level` | Significant roll/pitch | Fins active; thruster low |
| `point` | Apple off-centre | Aft Y/Z tracking; thruster gated |
| `approach` | Aligned + drive requested | Thruster rises |
| `idle` | No apple or no motion | Actuators near zero |

Phases are computed in code but **not yet shown** on the dashboard header (future UI).

### ESP commands sent

```
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>
B <fore> <aft>
```

### Tests

```bash
python -c "from tests.test_sub_motion import *; test_fins_counter_roll(); test_ballast_apple_below_fills(); print('ok')"
```

Or run `tests/test_sub_motion.py` with pytest when installed.

---

## 16. Sub Dashboard & API

Open **`http://<pi-ip>:8080/sub/`** after starting `sub_server.py` or `inference.py` (with `interface: sub` or `--sub`).

### During inference (auto mode)

When YOLO is running and sub mode is **auto**:

| Panel | What updates |
|-------|--------------|
| **Camera POV** | Live MJPEG from inference at **`/video_feed`** (~15 FPS in dashboard panel) |
| **Gyro** | Live pitch/roll/yaw; horizon ring tilts with sub attitude |
| **Actuator bars** | Effective outputs: Aft Y, Aft Z, Thruster, Fin L, Fin R |
| **Ballast** | `cmd ±X · FILL/DRAIN` from ESP telemetry when YOLO commands height trim |
| **Serial monitor** | Live `S2 …` and `B …` TX lines plus `TEL …` RX |
| **Mode badge** | May show browser-saved mode — click **Auto (YOLO)** if needed |

Example serial during approach with apple below centre:

```
B 0.280 0.280
S2 0.120 -0.350 F -0.350 0.350 X 0.450
TEL ballast fore 0.412 2048 1 FILL
TEL gyro 2.1 -8.4 0.0
```

### Key routes

| Route | Method | Purpose |
|-------|--------|---------|
| `/sub/` | GET | Dashboard HTML |
| `/sub/api/stream` | GET | **SSE live feed** — all dashboard data pushed on state change |
| `/sub/api/status` | GET | ESP/Xbox connection, leak alarm, control mode |
| `/sub/api/telemetry` | GET | Aggregated battery, gyro, depth, leaks, ballast |
| `/sub/api/control` | GET/POST | Control mode + manual actuator values |
| `/sub/api/actuators` | GET | Effective/auto/manual/xbox-mapped actuators |
| `/sub/api/control/ballast` | POST | Ballast fill/drain/stop or raw fore/aft values |
| `/sub/api/ballast/calibrate` | POST | Send `CAL B <tank> top\|bottom` to ESP |
| `/sub/api/serial` | GET/POST | Serial log / send raw command line |
| `/sub/api/pins` | GET | Expected vs reported ESP pin map |
| `/sub/api/diagnostics` | GET | PING/PINS/TEST state |
| `/sub/api/test` | POST | Send diagnostic command (PING, TEST S, etc.) |
| `/sub/api/test/run` | POST | Run full pin confirmation checklist |
| `/sub/api/xbox` | GET | Raw Xbox stick/button state |
| `/video_feed` | GET | MJPEG camera (~15 FPS when camera is running) |
| `/snapshot` | GET | Single JPEG frame (debug; dashboard uses `/video_feed`) |

Full request/response shapes and SSE payload: **`docs/sub_endpoints_plan.md`**.

### Live updates (SSE)

The dashboard opens one **`EventSource`** to `/sub/api/stream`. The server pushes an `update` event whenever ESP telemetry, Xbox input, serial traffic, or control state changes (rate-capped ~30 Hz). User actions still use REST `POST` endpoints. No client-side polling.

### Dashboard panels

| Panel | Purpose |
|-------|---------|
| **Telemetry** | Battery, depth, gyro horizon, leak grid |
| **Ballast** | Fore/aft level bars, fill/drain/stop, per-tank sliders, calibration |
| **Control** | Mode tabs (Manual / Xbox / Auto), stick viz, manual actuator sliders, live actuator bars |
| **Pin diagnostics** | Expected vs ESP-reported pins, PING/PINS/TEST checklist |
| **ESP Serial Monitor** | Live Pi ↔ ESP log (TX/RX) and raw command input |

### Serial monitor

The serial monitor at the bottom of the dashboard shows both directions:

| Colour | Direction | Examples |
|--------|-----------|----------|
| Green (`rx`) | ESP → Pi | `TEL battery …`, `TEL heartbeat …`, `OK PONG` |
| Blue (`tx`) | Pi → ESP | `S2 …`, `B …` |
| Yellow (`sys`) | Bridge events | Port open/close, read/write errors |

**UI behaviour:**

- **Taller log** — about half the viewport (min 320px, max 520px) for easier reading.
- **Smart scroll** — auto-follows new lines only while you are scrolled to the bottom.
- **Scroll up or select text** — live feed pauses (hint: “paused — scroll to bottom to resume live feed”) so you can copy lines without the view jumping.
- **200-line buffer** — `GET /sub/api/serial?limit=200`.

Send raw ESP commands in the input box (e.g. `PING`, `PINS`, `HELP`). Line ending is added automatically.

### Control persistence (sliders)

The dashboard keeps your control choices across normal page reloads:

1. **Saved in `sessionStorage`** — mode, all manual actuator sliders, fore/aft ballast slider values.
2. **Restored on load** — before the SSE stream connects, saved values are applied to the UI and POSTed to `/sub/api/control`, `/sub/api/control/actuators`, and `/sub/api/control/ballast`.
3. **Stream does not move sliders** — live updates refresh actuator **bars** only; slider positions stay where you set them.
4. **Server re-sync** — if the backend mode drifts (e.g. after `sub_server.py` restart), the dashboard re-applies your saved mode and values.
5. **Ballast buttons** — Fill / Drain / Stop update both the server command and the matching slider + saved state.

To reset everything: hard refresh the tab or clear site data for the Pi’s origin.

### Bench testing (ESP with nothing wired)

With `sub_rc` running and no sensors/motors connected, a healthy link looks like this in the serial monitor:

```
TEL battery 13.20          ← floating ADC (not a real voltage)
TEL gyro 0.00 0.00 0.00    ← no IMU — firmware reports zeros
TEL depth 3.30             ← floating ADC (~3.3 V), not real depth
TEL leak 0 0 0 0           ← OK (leak pin disabled or pulled low)
TEL ballast fore 1.000 4095 0 STOP   ← pot ADC pinned high
TEL ballast aft 0.015 62 0 STOP      ← pot ADC near zero
TEL ballastcal fore -1 -1 0          ← not calibrated yet
TEL status READY
TEL fault NONE
TEL heartbeat 651          ← incrementing = live telemetry
B 0.000 0.000              ← Pi → ESP (ballast stop)
S2 0.000 0.000 F 0.000 0.000 X 0.000   ← Pi → ESP (actuators idle in manual)
```

| Signal | Bench (nothing wired) | Meaning |
|--------|----------------------|---------|
| `TEL status READY` + rising `heartbeat` | ✓ | Protocol and firmware OK |
| `TEL leak 0 0 0 0` | ✓ | No leak alarm |
| `TEL gyro 0 0 0` | ✓ | No IMU wired |
| `B 0 0`, `S2 … 0` | ✓ | Pi sending idle commands (manual mode) |
| `TEL battery` / `depth` / `ballast` levels | Ignore | Floating ADC pins — meaningless until wired and calibrated |

Quick check: type **`PING`** in the serial monitor input → expect **`OK PONG`** in green.

---

## 17. ESP32 Telemetry & Commands

The ESP bridge (`src/esp_bridge.py`) runs background threads on the serial port configured in **`sub_serial`**:

- **Reader:** parses `TEL` telemetry and diagnostic replies into `sub_state`
- **Writer:** sends ballast + actuator commands at ~20 Hz (paused during diagnostic mode)
- **Stale watcher:** marks telemetry disconnected after 3 s silence

**Port resolution** (automatic, in order):

1. `/dev/serial/by-id/*` matching Espressif / ESP32 / USB JTAG
2. `sub_serial.port` from config if the path exists
3. First `/dev/ttyACM*` or `/dev/ttyUSB*`

Override at runtime: `--serial-port /dev/ttyACM1` on `inference.py` or `sub_server.py`.

USB CDC gets a **4 s boot grace** after open (ESP32-S3 reset lines). Boot spam (`ESP-ROM:`, `rst:`) is filtered from the serial log.

### Pi → ESP commands

```
B <fore> <aft>\n
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>\n
CAL B <fore|aft> top|bottom|show
PING
PINS
TEST S <ch 0-3> <val -1..1>
TEST T <val -1..1>
TEST B <fore|aft> fill|drain|stop
TEST L
TEST A
HELP
```

### ESP → Pi telemetry

```
TEL battery 12.45
TEL gyro <pitch> <roll> <_yaw>
TEL depth 3.45
TEL leak 0 0 1 0
TEL ballast fore 0.500 2048 1 FILL
TEL ballast aft 0.400 1638 0 STOP
TEL controls          (+ 7 value lines follow)
TEL thruster 0.400 128
TEL status READY
TEL fault NONE
TEL heartbeat 42
```

Telemetry is considered stale after 3 s with no updates.

### Utility scripts

```bash
# Probe GPIO UART (listen + PING)
python scripts/probe_esp_uart.py

# Simulate ESP on a PTY (full Pi serial path test)
python scripts/simulate_esp_serial.py

# Fake telemetry for UI development
python scripts/test_telemetry.py
```

See **`docs/ESP32_SERIAL.md`** for protocol details and **`esp32/README.md`** for firmware upload.

---

## 18. Complete Command Reference

Quick index of every runnable entry point. All Python commands assume:

```bash
cd ~/yolo-project
source .venv/bin/activate
```

### 18.1 `inference.py` — YOLO detection loop

**Purpose:** Main Pi runtime. Camera → YOLO → tracker → controller → hardware/sub bridge. Optionally serves web UI and sub dashboard.

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--device` | int | `0` | USB camera V4L2 index (`/dev/videoN`) |
| `--width` | int | `640` | Capture width (pixels) |
| `--height` | int | `480` | Capture height (pixels) |
| `--headless` | flag | off | Skip OpenCV display window |
| `--print-on-detect` | flag | off | Print ControlOutput table only when apple detected |
| `--quiet` | flag | auto when `--web` or sub active | Suppress per-frame terminal table |
| `--timing` | flag | off | Every 30 frames: log `cap`/`yolo`/`draw` ms and HUD FPS |
| `--tracker-strategy` | choice | `best_confidence` | `best_confidence` or `closest_to_centre` when multiple apples |
| `--apple-label` | str | `apple` | YOLO class name to track |
| `--web` | flag | off | Start Flask server; MJPEG at `/video_feed`, index at `/` |
| `--web-host` | str | `0.0.0.0` | HTTP bind address |
| `--web-port` | int | `8080` | HTTP port |
| `--serial-port` | str | from config | Override `sub_serial.port` for ESP bridge |
| `--control-profile` | choice | `config` | `config`, `stable`, or `aggressive` |
| `--sub` | flag | auto when `interface: sub` | Force sub dashboard + ESP bridge + Xbox |
| `--no-sub` | flag | off | Disable sub stack even when `interface: sub` |
| `--no-xbox` | flag | off | Skip Xbox controller polling |
| `--tolerate-missing-devices` | flag | off | Continue if camera/serial missing; manual-only fallback |

**Common invocations:**

```bash
# Production (interface: sub in hardware.yaml)
python inference.py --web --timing

# YOLO only, local window, no hardware
# Set interface: stub in config/hardware.yaml first
python inference.py

# SSH headless, no browser
python inference.py --headless --quiet

# Force sub off (YOLO + web only)
python inference.py --web --no-sub

# Bench: keep server up without camera
python inference.py --web --tolerate-missing-devices
```

**Exit:** Press **q** in OpenCV window, or **Ctrl+C** in terminal.

---

### 18.2 `sub_server.py` — Sub dashboard without YOLO

**Purpose:** ESP telemetry, actuator/ballast control, Xbox input, camera MJPEG — no apple detection.

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--host` | str | `0.0.0.0` | HTTP bind address |
| `--port` | int | `8080` | HTTP port |
| `--serial-port` | str | from config | Override ESP serial port |
| `--no-camera` | flag | off | Skip USB camera (placeholder MJPEG) |
| `--camera-device` | int | `0` | Camera V4L2 index |
| `--no-xbox` | flag | off | Skip Xbox polling |
| `--no-esp` | flag | off | Skip ESP bridge (use with `test_telemetry.py` instead) |

```bash
python sub_server.py
python sub_server.py --serial-port /dev/ttyACM0 --no-xbox
python sub_server.py --no-camera --port 9090
```

Open **`http://<pi-ip>:8080/sub/`**. Default control mode: **manual** (sliders).

---

### 18.3 `train.py` — Model training (GPU / Colab)

**Purpose:** Train YOLO on apple dataset. **Do not run on Pi** — use Colab or a GPU machine.

| Flag | Type | Default | Description |
|------|------|---------|-------------|
| `--epochs` | int | from `model.yaml` | Training epochs |
| `--batch` | int | from `model.yaml` | Batch size |
| `--model` | str | from `model.yaml` | Architecture string |
| `--imgsz` | int | from `model.yaml` | Input image size |
| `--resume` | flag | off | Resume from `weights:` in `model.yaml` |
| `--device` | str | auto | `cpu`, `0`, `mps`, etc. |

```bash
python train.py
python train.py --epochs 100 --batch 8 --model yolov8s
```

---

### 18.4 `python -m src.xbox_controller` — Gamepad diagnostic

**Purpose:** List connected gamepads and print live stick/button → actuator mapping without starting the full server.

```bash
python -m src.xbox_controller
```

- Enumerates pads via pygame/SDL
- Connects for ~10 s or until a pad opens
- Prints LS/RS, triggers, mapped thruster/fins/ballast
- **Ctrl+C** to quit

Requires `xbox.enabled: true` in `config/hardware.yaml` (warns if false).

---

### 18.5 Shell scripts (project root)

| Script | Command | Purpose |
|--------|---------|---------|
| **`scripts/check_camera.sh`** | `bash scripts/check_camera.sh` | Lists `/dev/video*`, user groups, `v4l2-ctl` devices, `lsusb`, recent `dmesg`, processes holding camera |
| **`scripts/zip_dataset.sh`** | `bash scripts/zip_dataset.sh` | Creates **`dataset.zip`** from `data/images/` for Colab upload |
| **`esp32/upload_from_pi.sh`** | See **§21** | Build + flash ESP32 firmware from Pi |

---

### 18.6 Python scripts in `scripts/`

| Script | Purpose |
|--------|---------|
| **`scripts/test_telemetry.py`** | Fake ESP telemetry + dashboard (no hardware) |
| **`scripts/simulate_esp_serial.py`** | Fake ESP on PTY; tests full serial round-trip |
| **`scripts/probe_esp_uart.py`** | Listen + PING on UART ports |

Detailed usage: **§19**.

---

## 19. Scripts & Utilities

### 19.1 `scripts/test_telemetry.py`

**Simulation only** — populates `sub_state` with fake battery, gyro, depth, leaks, ballast, and Xbox data. Does **not** open real serial.

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | `0.0.0.0` | HTTP bind |
| `--port` | `8080` | HTTP port |
| `--hz` | `5.0` | Simulated telemetry update rate |
| `--no-camera` | off | Skip USB camera feed |
| `--camera-device` | `0` | Camera index |
| `--leak-alarm` | off | Toggle leak sensor 3 every 20 s (UI alarm test) |

```bash
python scripts/test_telemetry.py
python scripts/test_telemetry.py --port 8080 --leak-alarm
python scripts/test_telemetry.py --no-camera
```

Use when developing the dashboard UI without ESP32 or when demoing leak alarms.

---

### 19.2 `scripts/simulate_esp_serial.py`

**Full serial path test** — pretends to be `sub_rc.ino`: sends `TEL` lines, parses `B` / `S2` / `CAL` from Pi.

| Flag | Default | Description |
|------|---------|-------------|
| `--pi-port` | `/tmp/sub_fake_pi` | Path for `sub_server --serial-port` |
| `--esp-port` | `/tmp/sub_fake_esp` | Path this script opens (socat mode) |
| `--hz` | `5.0` | Telemetry transmit rate |
| `--leak-alarm` | off | Toggle simulated leak every 20 s |

**Two-terminal workflow:**

```bash
# Terminal 1
python scripts/simulate_esp_serial.py

# Terminal 2 (use port printed by Terminal 1)
python sub_server.py --serial-port /tmp/sub_fake_pi --no-xbox
```

Uses **socat** if installed (creates linked PTYs); otherwise falls back to a single PTY symlink. Move dashboard sliders to see `<< (Pi cmd) B …` / `S2 …` in Terminal 1.

---

### 19.3 `scripts/probe_esp_uart.py`

**Hardware probe** — opens serial, listens for spontaneous `TEL` lines, sends `PING`, reports `OK PONG`.

| Flag | Default | Description |
|------|---------|-------------|
| `--port` | `/dev/ttyAMA0`, `/dev/ttyAMA10` | Repeatable; probes each port listed |

```bash
python scripts/probe_esp_uart.py
python scripts/probe_esp_uart.py --port /dev/ttyACM0 --port /dev/serial0
```

Exit code `0` if any port responds with `OK PONG` or spontaneous `TEL`. Prints wiring checklist on failure.

---

### 19.4 `scripts/check_camera.sh`

Non-destructive camera diagnostics. No arguments.

```bash
bash scripts/check_camera.sh
```

Checks:

1. `/dev/video*` device nodes
2. Current user groups (`video` group needed)
3. `v4l2-ctl --list-devices` (install: `sudo apt install v4l-utils`)
4. `lsusb` for camera presence
5. Recent kernel messages
6. Processes using video devices (`fuser`)

**Manual MJPEG test:**

```bash
vlc v4l2:///dev/video0 --v4l2-chroma=MJPG
```

---

### 19.5 `scripts/zip_dataset.sh`

Packages training data for Colab upload.

```bash
bash scripts/zip_dataset.sh
```

Creates **`dataset.zip`** from **`data/images/`** (must contain `data.yaml`). Copy to PC or upload directly in Colab.

---

### 19.6 `train_colab.ipynb`

Self-contained Google Colab notebook at project root. Typical flow:

1. Upload **`dataset.zip`** and project files (or clone repo)
2. Install dependencies
3. Run training cells → produces **`weights/best.pt`**
4. Download weights to Pi

Alternative to manual Colab steps in **§5**.

---

## 20. Configuration Files Reference

All YAML lives in **`config/`**. Paths are relative to project root unless absolute.

### 20.1 `config/model.yaml`

| Key | Example | Used by | Description |
|-----|---------|---------|-------------|
| `architecture` | `yolov8n` | `train.py`, `inference.py` | Ultralytics model name |
| `weights` | `weights/best.pt` | `inference.py` | Trained `.pt` path; empty = pretrained backbone only |
| `confidence` | `0.50` | `Detector` | Detection confidence threshold |
| `iou` | `0.45` | `Detector` | NMS IoU threshold |
| `img_size` | `640` | train + infer | Square input size |
| `epochs` | `50` | `train.py` | Default training epochs |
| `batch_size` | `16` | `train.py` | Default batch size |

---

### 20.2 `config/dataset.yaml`

| Key | When | Description |
|-----|------|-------------|
| `source` | always | `local` or `roboflow` |
| `local_path` | `source: local` | Folder containing `data.yaml` (e.g. `data/images`) |
| `workspace`, `project`, `version`, `format` | `source: roboflow` | Roboflow export settings |
| `api_key` | roboflow | Leave blank; set env `ROBOFLOW_API_KEY` |

---

### 20.3 `config/hardware.yaml`

| Section | Keys | Description |
|---------|------|-------------|
| **`interface`** | `stub` \| `sub` | `stub` = log only; `sub` = route to ESP via `SubBridgeOutput` |
| **`sub_serial`** | `port`, `baud_rate` | ESP link (default USB `/dev/ttyACM0`, 115200) |
| **`xbox`** | `enabled`, `deadzone`, `trigger_deadzone`, `smoothing_alpha`, `stick_hold_ms`, `release_alpha`, `poll_hz`, `scan_interval_s`, `mapping_file`, `device_index` | Gamepad polling (**§14**) |
| **Control** | `deadzone`, `gain_steer`, `gain_tilt`, `max_drive_speed`, `min_steer_command`, `min_tilt_command`, `min_drive_command`, `min_detection_confidence`, `hold_missed_frames`, `smoothing_alpha`, `damping_steer`, `damping_tilt`, `max_drive_when_turning` | YOLO → ControlOutput tuning |
| **Size drive** | `use_size_for_drive`, `size_drive_far`, `size_drive_close`, `size_min_ratio`, `size_max_ratio`, `size_smoothing_alpha`, `size_curve` | Bbox area → forward speed |
| **`approach`** | `use_telemetry`, `stop_on_leak`, `min_battery_v`, `low_battery_drive_scale`, `max_error_x_for_drive`, `max_error_y_for_drive` | Telemetry safety on drive |
| **`sub_motion`** | `level_roll_gain`, `level_pitch_gain`, `max_fin_command`, `max_roll_deg`, `max_pitch_deg`, `require_level_for_steer`, `require_level_for_drive`, `use_ballast_for_height`, `ballast_height_gain`, `ballast_error_deadzone`, `ballast_max_command` | Layered auto actuators (**§15**) |
| **`cameras`** | `num_cameras` | Placeholder for future multi-cam (1–3) |

---

### 20.4 `config/xbox_mapping.yaml`

Gamepad → actuator bindings. Loaded by `src/xbox_mapping.py`. Edit without changing Python.

| Section | Purpose |
|---------|---------|
| **`sticks.left`** | `aft_steer_y: x`, `aft_steer_z: y`, `invert_y`, optional `deadzone` |
| **`sticks.right`** | `mode: polar_fins`, `snap_degrees` — fin angles from stick bearing |
| **`buttons`** | SDL button indices: `a`, `b`, `lb`, etc. |
| **`triggers`** | `lt: fill` — analog fill |
| **`ballast`** | `drain_button`, `fill_trigger`, `dpad_up_tank`, `dpad_down_tank`, `default_tank` |
| **`thruster`** | `forward_button: rb`, `reverse_button: rt`, magnitudes |

Verify: `python -m src.xbox_controller`

---

### 20.5 `config/pins.yaml`

**Reference only** — not read at runtime yet. Documents GPIO, PCA9685 channels, L298N, ballast, leak pins synced from `esp32/sub_rc/sub_rc.ino`. Used by `/sub/api/pins` “expected” snapshot.

---

## 21. ESP32 Firmware & Upload

### 21.1 Available sketches

| Folder | Purpose |
|--------|---------|
| **`esp32/sub_rc/`** | **Production** — actuators, ballast, sensors, `TEL` telemetry, diagnostics |
| **`esp32/sub_hw_test/`** | Isolated hardware bring-up (motors/servos one at a time) |

Flash **`sub_rc`** for normal operation.

### 21.2 `esp32/upload_from_pi.sh`

Build and upload from the Pi (ESP32 connected via **USB** for flashing).

```bash
# List USB ports
bash esp32/upload_from_pi.sh scan

# Auto-detect port, build sub_rc
bash esp32/upload_from_pi.sh

# Explicit board + port + sketch
bash esp32/upload_from_pi.sh esp32:esp32:esp32s3 /dev/ttyACM0 sub_rc

# Hardware test sketch
bash esp32/upload_from_pi.sh --sketch sub_hw_test

# Named sketch as third argument
bash esp32/upload_from_pi.sh esp32:esp32:esp32s3 /dev/ttyACM0 sub_hw_test
```

| Argument | Default | Description |
|----------|---------|-------------|
| `scan` / `list` / `--list-ports` | — | List ports and exit |
| `--sketch NAME` | `sub_rc` | Sketch folder under `esp32/` |
| `[BOARD]` | `esp32:esp32:esp32s3` | Arduino FQBN |
| `[PORT]` | auto-detect | USB serial for upload |
| `[SKETCH]` | `sub_rc` | Third positional sketch name |

**Prerequisites:** `arduino-cli`, ESP32 core, libraries — see **`esp32/README.md`**.

**Note:** Upload uses USB. Runtime serial to Pi may be the same USB cable (`/dev/ttyACM0`) or GPIO UART — set `sub_serial.port` accordingly.

### 21.3 Firmware serial behaviour

- **Baud:** 115200
- **Timeout:** 8000 ms — thruster stops if Pi stops sending
- **Telemetry:** ~5 Hz `TEL` lines
- **Diagnostics:** `PING`, `PINS`, `TEST …`, `CAL B …`, `HELP`
- **USB debug:** `USB_DEBUG_MIRROR=1` in sketch mirrors key messages to USB Serial Monitor

### 21.4 Pi UART enable (GPIO wiring only)

If using GPIO UART instead of USB:

1. Enable UART in `/boot/firmware/config.txt` (`dtparam=uart0=on` on Pi 5)
2. Disable serial console if it occupies the port
3. Set `sub_serial.port: "/dev/serial0"`
4. Flash `sub_rc` with `USE_PI_UART=1` in the sketch

Probe: `python scripts/probe_esp_uart.py --port /dev/serial0`

---

## 22. Web Routes & HTTP API

Default base URL: **`http://<pi-ip>:8080`** (port from `--web-port` / `--port`).

### 22.1 YOLO / camera routes (`web_stream.py`)

Started by `inference.py --web` or any process that calls `run_server()`.

| Route | Method | Description |
|-------|--------|-------------|
| `/` | GET | Minimal HTML page linking to stream and `/sub/` |
| `/video_feed` | GET | MJPEG multipart stream (~15 FPS when inference running) |
| `/snapshot` | GET | Single JPEG frame (debug); same source as video feed |

There is **no** separate `/api/control` for YOLO S/D/T — manual/auto for the sub stack is entirely under **`/sub/api/*`**.

### 22.2 Sub dashboard routes (`sub_web.py`)

See **§16** for narrative docs. Summary:

**Pages:** `/sub/`, `/sub`

**Read (GET):**

| Route | Returns |
|-------|---------|
| `/sub/api/status` | Connection flags, mode, telemetry age |
| `/sub/api/telemetry` | Full telemetry snapshot |
| `/sub/api/telemetry/battery` | `{ voltage, connected, timestamp }` |
| `/sub/api/telemetry/gyro` | `{ pitch, roll, yaw, connected }` |
| `/sub/api/telemetry/depth` | `{ meters, connected }` |
| `/sub/api/telemetry/leaks` | `{ sensors[], triggered, connected }` |
| `/sub/api/telemetry/ballast` | Fore/aft levels and commands |
| `/sub/api/control` | Mode, effective/auto/manual/xbox actuators, ballast commands |
| `/sub/api/actuators` | Same actuator breakdown |
| `/sub/api/xbox` | Raw pad state |
| `/sub/api/serial?limit=N` | Serial log lines (1–500) |
| `/sub/api/pins` | Expected vs ESP pin map |
| `/sub/api/diagnostics` | Last PONG, PINS capture, test replies |
| `/sub/api/stream` | **SSE** — live combined dashboard payload |

**Write (POST):**

| Route | Body (JSON) | Effect |
|-------|-------------|--------|
| `/sub/api/control` | `{ "mode": "auto"\|"manual"\|"xbox", "aftSteerY": 0.5, ... }` | Set mode and/or manual actuators |
| `/sub/api/control/actuators` | `{ "aftSteerY", "thrusterX", "finLeft", ... }` | Manual actuators (forces manual) |
| `/sub/api/control/ballast` | `{ "action": "fill"\|"drain"\|"stop", "tank": "fore"\|"aft"\|"both" }` or `{ "fore": 1.0, "aft": -1.0 }` or `{ "value": 0.5, "tank": "fore" }` | Ballast command |
| `/sub/api/ballast/calibrate` | `{ "tank": "fore", "end": "top"\|"bottom" }` | Send `CAL B` to ESP |
| `/sub/api/ballast/calibrate/resume` | — | Ack resume (no-op) |
| `/sub/api/serial` | `{ "line": "PING" }` | Send raw command (newline added) |
| `/sub/api/test` | `{ "command": "PING" }` | Diagnostic mode + single command |
| `/sub/api/test/resume` | — | Exit diagnostic mode |
| `/sub/api/test/run` | — | Run full pin checklist |

**Actuator JSON keys:** `aftSteerY`, `aftSteerZ`, `thrusterX`, `finLeft`, `finRight` (snake_case aliases accepted).

**curl examples:**

```bash
curl http://localhost:8080/sub/api/status
curl -X POST http://localhost:8080/sub/api/control \
  -H 'Content-Type: application/json' \
  -d '{"mode":"manual","thrusterX":0.3}'
curl -X POST http://localhost:8080/sub/api/control/ballast \
  -H 'Content-Type: application/json' \
  -d '{"action":"fill","tank":"fore"}'
curl -X POST http://localhost:8080/sub/api/serial \
  -H 'Content-Type: application/json' \
  -d '{"line":"PING"}'
curl -X POST http://localhost:8080/sub/api/test/run
```

Full JSON schemas: **`docs/sub_endpoints_plan.md`**.

---

## 23. Tests & Verification

### 23.1 `tests/test_sub_motion.py`

Unit tests for layered auto control (`src/sub_motion.py`).

```bash
# Run with pytest (if installed)
pytest tests/test_sub_motion.py -v

# Or run individual checks
python -c "
from tests.test_sub_motion import (
    test_fins_counter_roll,
    test_thruster_gated_when_rolled,
    test_ballast_apple_below_fills,
)
test_fins_counter_roll()
test_thruster_gated_when_rolled()
test_ballast_apple_below_fills()
print('ok')
"
```

| Test | Verifies |
|------|----------|
| `test_fins_counter_roll` | Positive roll → differential fin correction |
| `test_thruster_gated_when_rolled` | High roll reduces thruster |
| `test_ballast_apple_below_fills` | Positive `error_y` → fill command |

### 23.2 End-to-end checklists

**YOLO + camera:**

```bash
bash scripts/check_camera.sh
python inference.py --headless --timing --no-sub   # if interface: stub
```

**ESP serial (real hardware):**

```bash
python scripts/probe_esp_uart.py --port /dev/ttyACM0
python sub_server.py
# Dashboard → serial monitor → PING → OK PONG
curl -X POST http://localhost:8080/sub/api/test/run
```

**ESP serial (simulated):**

```bash
# Terminal 1: python scripts/simulate_esp_serial.py
# Terminal 2: python sub_server.py --serial-port /tmp/sub_fake_pi --no-xbox
```

**Dashboard UI (no hardware):**

```bash
python scripts/test_telemetry.py --leak-alarm
```

**Xbox:**

```bash
python -m src.xbox_controller
python sub_server.py   # then open /sub/ and switch to Xbox mode
```

---

## Related docs

- **`README.md`** — Quick start
- **`STATUS.md`** — Current project status
- **`docs/ESP32_SERIAL.md`** — Serial protocols (S/D/T + sub)
- **`docs/PI_ESP32_COMPATIBILITY.md`** — Pi ↔ ESP32 checklist
- **`docs/sub_endpoints_plan.md`** — Sub API reference
- **`esp32/README.md`** — Firmware build and upload
- **`config/pins.yaml`** — Pin and channel map
