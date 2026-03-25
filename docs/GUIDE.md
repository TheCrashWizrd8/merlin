# YOLO Apple RC Car — Full System Guide

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

---

## 1. System Overview

The system detects apples in a live USB camera feed using Ultralytics YOLO, tracks the best apple, and outputs normalised **steering**, **drive**, and **camera tilt** commands. The Raspberry Pi can drive **servos and a DC motor** directly (PCA9685 / GPIO) or send commands over **USB serial** to an **ESP32** (e.g. L298N motor + servos).

```
USB Camera ──► Detector ──► Tracker ──► Controller ──► ControlOutput
                                              │
                    ┌─────────────────────────┼─────────────────────────┐
                    │                         │                         │
              steering_servo            drive_motor            camera_tilt_servo
                    │                         │                         │
                    └─────────────────────────┼─────────────────────────┘
                                              │
                         stub / PCA9685 / GPIO / serial (ESP32)
```

### Goal behaviour (high level)

| Situation | Behaviour |
|-----------|-------------|
| Apple left / right of centre | Steering corrects (sustain-until-centred; see `hardware.yaml`) |
| Apple above / below centre | Camera tilt corrects |
| Apple far (small in frame) | Stronger forward drive (when size-based drive is enabled) |
| Apple close (large in frame) | Weaker forward drive |
| Lost / low confidence | Outputs decay to stop; brief hold of last valid target |

---

## 2. Hardware Setup

### Typical stack (Pi + ESP32)

| Component | Role |
|-----------|------|
| Raspberry Pi 5 | Main compute (YOLO + control loop) |
| USB camera | Live input |
| ESP32-S3 (USB serial to Pi) | PWM servos + L298N motor driver |
| Hitec HS-646WP (×2) | Steering + camera tilt |
| L298N + DC motor | Drive |

Set `interface: serial` in `config/hardware.yaml`. Firmware and wiring: **`esp32/README.md`**, **`docs/ESP32_SERIAL.md`**, **`docs/PI_ESP32_COMPATIBILITY.md`**.

### Other interfaces (`config/hardware.yaml`)

| `interface` | Use case |
|-------------|----------|
| `stub` | No hardware; prints commands only (development) |
| `pca9685` | Adafruit PCA9685 I2C PWM (servos on Pi) |
| `gpio` | Direct Pi PWM (if implemented for your pins) |
| `serial` | Text protocol `S` / `D` / `T` to ESP32 or similar |

### PCA9685 wiring (when using that interface)

```
Pi 5  SDA (GPIO 2) ──► PCA9685 SDA
Pi 5  SCL (GPIO 3) ──► PCA9685 SCL
Pi 5  3.3V         ──► PCA9685 VCC
Pi 5  GND          ──► PCA9685 GND
Separate 5V supply ──► PCA9685 V+ (servo power rail)
```

---

## 3. Software Architecture

### Module responsibilities

| File | Responsibility |
|------|----------------|
| `src/detector.py` | Ultralytics YOLO; returns `Detection` with bbox + class |
| `src/tracker.py` | Picks target apple; bbox centre; `error_x` / `error_y`; bbox size |
| `src/controller.py` | `TrackResult` → `ControlOutput`; smoothing; size-based drive |
| `src/hardware.py` | Maps `ControlOutput` to stub / PCA9685 / serial |
| `src/camera.py` | USB camera via OpenCV |
| `src/display.py` | Bbox, crosshair, HUD, gauges |
| `src/control_source.py` | Manual vs auto (e.g. web sliders vs inference) |
| `src/web_stream.py` | Flask MJPEG + `/api/control` when using `--web` |
| `src/dataset.py` | Resolves `data.yaml` (local or optional Roboflow download) |
| `train.py` | Training on GPU/Colab (not on Pi) |
| `inference.py` | Main loop on Pi |

### Data flow per frame

```
cam.read()
    └──► detector.detect(frame)     → List[Detection]
              └──► tracker.update(...)  → TrackResult
                        └──► controller.compute(track)  → ControlOutput
                                  ├──► hardware.apply(output)
                                  ├──► output.pretty()  → terminal
                                  └──► display.draw(frame, output) → annotated view
```

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

```bash
python train.py --model yolov8s
python train.py --epochs 100
python train.py --batch 8
python train.py --resume
python train.py --device cpu
```

### After training

1. Copy **`weights/best.pt`** to the Pi (USB, `scp`, or download from Colab).
2. Update `config/model.yaml`: `weights: weights/best.pt`
3. Run `python inference.py` on the Pi.

---

## 7. Running Inference on the Pi

### With display

```bash
python inference.py
```

Press **q** to quit.

### Headless (SSH)

```bash
python inference.py --headless
```

### Web UI (`--web`)

```bash
python inference.py --web
```

Open `http://<pi-ip>:8080` (or `--web-port`). Live MJPEG + control API; see `src/web_stream.py`.

### Inference flags

| Flag | Default | Description |
|------|---------|-------------|
| `--device N` | `0` | USB camera index |
| `--width` / `--height` | `640` / `480` | Capture size |
| `--headless` | off | No OpenCV window |
| `--print-on-detect` | off | Print only when apple detected |
| `--tracker-strategy` | `best_confidence` | `best_confidence` or `closest_to_centre` |
| `--apple-label` | `apple` | Class label to track |
| `--web` | off | Flask server + browser stream |
| `--web-host` / `--web-port` | `0.0.0.0` / `8080` | Bind address and port |
| `--serial-port` | config | Override e.g. `/dev/ttyACM1` |
| `--serial-verbose` / `--no-serial-verbose` | echo S D T | Console serial echo |
| `--control-profile` | `config` | `config`, `stable`, or `aggressive` |

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
| `timestamp` | `time.time()` |

### Behaviour (summary)

- **Smoothing** on errors (`smoothing_alpha` in `hardware.yaml`).
- **Sustain-until-centred** steering/tilt: constant command until error inside deadzone (not a gentle proportional ramp).
- **Size-based drive** (optional): `use_size_for_drive` maps apparent apple size to **D** — typically **far** (small bbox) → higher `size_drive_far`, **close** (large bbox) → lower `size_drive_close`. Tune `size_min_ratio` / `size_max_ratio` to match HUD **Size** values.

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

- **`interface`**: `stub` | `pca9685` | `gpio` | `serial`
- **`serial`**: `port`, `baud_rate`, `verbose` (ESP32 USB)
- **`cameras.num_cameras`**: 1–3 (placeholder for future multi-camera)
- **Control**: `deadzone`, `min_steer_command`, `min_tilt_command`, `min_drive_command`, `smoothing_alpha`, `confidence`, `hold_missed_frames`, etc.
- **Size-based drive**: `use_size_for_drive`, `size_drive_far`, `size_drive_close`, `size_min_ratio`, `size_max_ratio`, `size_smoothing_alpha`, `size_curve` (`sqrt` or `linear`)

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

The project already implements hardware output. Use:

```python
from src.hardware import from_config as hardware_from_config

hardware = hardware_from_config(serial_port_override=args.serial_port)
hardware.apply(output)
# ...
hardware.close()
```

`inference.py` does this automatically. **Serial** sends lines:

`S <steer> D <drive> T <tilt>\n` with values in **-1.0 … +1.0**.

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

- Use `yolov8n`, lower `img_size`, `--headless` to skip GUI work.

### Display over SSH

Use `--headless` or `ssh -X` for X forwarding.

---

## Related docs

- **`README.md`** — Quick start
- **`docs/ESP32_SERIAL.md`** — Serial protocol
- **`docs/PI_ESP32_COMPATIBILITY.md`** — Pi ↔ ESP32 notes
- **`esp32/README.md`** — Firmware build and upload
