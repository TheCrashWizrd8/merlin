# YOLO Apple RC Car — Full System Guide

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Setup](#2-hardware-setup)
3. [Software Architecture](#3-software-architecture)
4. [Installation](#4-installation)
5. [Dataset Setup (Roboflow)](#5-dataset-setup-roboflow)
6. [Training the Model](#6-training-the-model)
7. [Running Inference on the Pi](#7-running-inference-on-the-pi)
8. [ControlOutput Reference](#8-controloutput-reference)
9. [Swapping the Dataset (Prod)](#9-swapping-the-dataset-prod)
10. [Swapping the YOLO Model](#10-swapping-the-yolo-model)
11. [Adding Hardware Control](#11-adding-hardware-control)
12. [Troubleshooting](#12-troubleshooting)

---

## 1. System Overview

The system detects apples in a live USB camera feed using a YOLO model and
outputs normalised servo/motor commands that would drive an RC car towards
the apple and keep it centred in the frame.

```
USB Camera ──► YOLO Detector ──► Tracker ──► Controller ──► ControlOutput
                                                                  │
                                                  ┌───────────────┼───────────────┐
                                           steering_servo   drive_motor   camera_tilt_servo
```

### Goal behaviour

| Apple position | Expected response |
|----------------|-------------------|
| Left of centre | Steer left, camera stays level |
| Right of centre | Steer right |
| Above centre | Camera tilts up to recentre |
| Below centre | Camera tilts down |
| Centred | Drive forward, small corrections only |
| Not visible | All outputs zero (stop) |

---

## 2. Hardware Setup

### RC Car components

| Component | Role |
|-----------|------|
| Raspberry Pi 5 (8 GB) | Main compute |
| USB camera | Input — plugged into any USB port |
| Steering servo | Controls left/right direction |
| Drive motor + ESC | Controls forward/reverse speed |
| Camera tilt servo | Tilts camera up/down on pivot |

### Wiring (to be completed when hardware arrives)

The `config/hardware.yaml` file has a commented `interface` field with four
options:

- `stub` — no hardware wired; all commands are printed to the terminal only.
  Use this during development.
- `pca9685` — Adafruit PCA9685 I2C PWM board (recommended; gives 16 independent
  PWM channels, powered separately from the Pi GPIO).
- `gpio` — direct Pi GPIO PWM via `RPi.GPIO` or `pigpio`.
- `serial` — send commands to an Arduino/microcontroller over UART.

Until the hardware layer module (`src/hardware.py`) is implemented the system
runs in `stub` mode and is fully functional for development.

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
|------|---------------|
| `src/detector.py` | Wraps Ultralytics YOLO; returns a list of `Detection` objects per frame |
| `src/tracker.py` | Picks the best apple; computes normalised `error_x` / `error_y` |
| `src/controller.py` | Converts errors to `ControlOutput`; pretty-prints the table |
| `src/camera.py` | Opens/reads USB camera via OpenCV `VideoCapture` |
| `src/display.py` | Draws bboxes, crosshair, error vector, HUD, and gauge bars on the frame |
| `src/dataset.py` | Downloads from Roboflow or resolves a local dataset path |
| `train.py` | Full training pipeline (run on a GPU machine or Colab) |
| `inference.py` | Main loop for the Pi — ties all modules together |

### Data flow per frame

```
cam.read()
    └──► detector.detect(frame)         → List[Detection]
              └──► tracker.update(...)  → TrackResult
                        └──► controller.compute(track)  → ControlOutput
                                  ├──► output.pretty()  → printed table
                                  └──► display.draw(frame, output) → annotated view
```

### ControlOutput normalisation

All output values are in the range **-1.0 … +1.0**.

```
-1.0 ──────────── 0.0 ──────────── +1.0
 full left       centre         full right   (steering)
 full rev        stop           full fwd     (drive)
 full down       level          full up      (tilt)
```

The hardware layer maps these to actual PWM microseconds using the `min_pwm`,
`centre_pwm`, and `max_pwm` values in `config/hardware.yaml`.

### Proportional control

The current controller uses a simple proportional (P) law:

```
steering_servo    =  gain_steer × error_x
camera_tilt_servo = −gain_tilt  × error_y   (inverted: apple low → tilt down)
drive_motor       =  gain_drive × (1 − |error_x|)   (slow down when turning)
```

A full PID controller can be added later by extending `src/controller.py`.

---

## 4. Installation

### On the Pi 5 (inference only)

```bash
# Update system packages
sudo apt update && sudo apt upgrade -y

# Install system-level OpenCV dependencies
sudo apt install -y libopencv-dev python3-opencv

# Create a virtual environment (recommended)
python3 -m venv .venv
source .venv/bin/activate

# Install Python dependencies
pip install -r requirements.txt
```

> **Note:** PyTorch for ARM64 is included in `ultralytics` via pip on
> Raspberry Pi OS (64-bit).  If you see errors about `torch` not being found,
> install it manually:
> ```bash
> pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
> ```

### On a training machine (GPU / Colab)

```bash
pip install -r requirements.txt
# CUDA PyTorch is pulled in automatically by ultralytics on CUDA machines.
```

For Google Colab, add this cell at the top:

```python
!pip install ultralytics roboflow python-dotenv pyyaml
import os
os.environ["ROBOFLOW_API_KEY"] = "your_key_here"
```

---

## 5. Dataset Setup (Roboflow)

### Getting your dataset

1. Go to [roboflow.com](https://roboflow.com) and create / find an apple
   detection dataset.
2. Note your **workspace slug**, **project slug**, and **version number**.
3. Get your **API key** from Account → Roboflow API.

### Configuring the project

Edit `config/dataset.yaml`:

```yaml
source: roboflow
workspace: "my-workspace-slug"
project: "apple-detection"
version: 1
format: yolov8
```

Set your API key as an environment variable (never commit it):

```bash
export ROBOFLOW_API_KEY="rf_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
```

Or create `.env` in the project root:

```
ROBOFLOW_API_KEY=rf_xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx
```

---

## 6. Training the Model

Training must be run on a machine with a GPU (or Google Colab).
The Pi5 is used for inference only.

### Basic training run

```bash
python train.py
```

This:
1. Downloads the dataset from Roboflow into `data/`
2. Trains `yolov8n` for 50 epochs
3. Saves the best weights to `weights/best.pt`
4. Runs validation and prints mAP metrics

### Common options

```bash
# Train a larger model
python train.py --model yolov8s

# More epochs
python train.py --epochs 100

# Smaller batch if GPU VRAM is limited
python train.py --batch 8

# Resume interrupted training
python train.py --resume

# Force CPU (slow but useful for testing on Pi)
python train.py --device cpu
```

### After training

1. Copy `weights/best.pt` to the Pi.
2. Update `config/model.yaml` on the Pi:

```yaml
weights: weights/best.pt
```

3. Run `inference.py`.

---

## 7. Running Inference on the Pi

### With a display (monitor or X forwarding)

```bash
python inference.py
```

A window opens showing the annotated camera feed.  Press **q** to quit.

### Headless (SSH, no display)

```bash
python inference.py --headless
```

The `ControlOutput` table is printed to the terminal every frame:

```
┌────────────────────────────────────────────────────────────┐
│  CONTROL OUTPUT                                  12:34:56.789 │
├───────────────────┬────────────────────────────────────────┤
│ apple_detected    │ YES                                    │
│ target            │ x=298   y=241                          │
│ error_x           │ -0.0688                                │
│ error_y           │ +0.0042                                │
│ confidence        │ 0.8731                                 │
├───────────────────┼────────────────────────────────────────┤
│ steering_servo    │ -0.069  ─────────|░░░░░░───  (left)   │
│ drive_motor       │ +0.557  ──────────|████████  (forward)│
│ camera_tilt       │ -0.003  ──────────|──────────  (centre)│
└───────────────────┴────────────────────────────────────────┘
```

### All inference flags

| Flag | Default | Description |
|------|---------|-------------|
| `--device N` | `0` | USB camera device index |
| `--width N` | `640` | Capture width |
| `--height N` | `480` | Capture height |
| `--headless` | off | Skip display window |
| `--print-on-detect` | off | Only print when apple is found |
| `--tracker-strategy` | `best_confidence` | `best_confidence` or `closest_to_centre` |
| `--apple-label STR` | `apple` | Class label to track |

---

## 8. ControlOutput Reference

```python
@dataclass
class ControlOutput:
    steering_servo:    float   # -1.0 = full left,    +1.0 = full right
    drive_motor:       float   # -1.0 = full reverse, +1.0 = full forward
    camera_tilt_servo: float   # -1.0 = full down,    +1.0 = full up
    apple_detected:    bool
    target_x:          int     # pixel x of apple centre
    target_y:          int     # pixel y of apple centre
    error_x:           float   # normalised horizontal offset from frame centre
    error_y:           float   # normalised vertical   offset from frame centre
    confidence:        float   # YOLO detection confidence 0–1
    timestamp:         float   # time.time()
```

### Accessing it in your own code

```python
from src.detector import Detector
from src.tracker import Tracker
from src.controller import Controller

detector   = Detector()
tracker    = Tracker()
controller = Controller.from_hardware_config()

# ... get a frame from Camera ...
detections = detector.detect(frame)
track      = tracker.update(detections, frame.shape)
output     = controller.compute(track)

# As a dataclass
print(output.steering_servo)

# As a plain dict (for JSON / serial transmission)
print(output.to_dict())

# Pretty-printed table
print(output.pretty())
```

---

## 9. Swapping the Dataset (Prod)

No code changes required — only a config change.

1. Place your prod dataset in e.g. `data/prod-apples/` (must contain `data.yaml`).
2. Edit `config/dataset.yaml`:

```yaml
source: local
local_path: "data/prod-apples"
```

3. Run `python train.py` to retrain, or just update `weights` in `config/model.yaml`
   if you already have trained weights.

---

## 10. Swapping the YOLO Model

Edit `config/model.yaml`:

```yaml
architecture: yolov8s   # was yolov8n
```

Model size vs Pi5 performance (approximate):

| Model | Params | Pi5 inference |
|-------|--------|---------------|
| yolov8n | 3.2 M | ~60–80 ms/frame |
| yolov8s | 11 M  | ~120–180 ms/frame |
| yolov8m | 25 M  | ~300 ms/frame |

For real-time performance on Pi5 (no NPU), `yolov8n` is recommended.
Consider reducing `img_size` to 320 if latency is too high.

---

## 11. Adding Hardware Control

When the hardware is wired up, create `src/hardware.py`:

```python
from src.controller import ControlOutput
import yaml
from pathlib import Path

class HardwareDriver:
    def __init__(self): ...          # read hardware.yaml, open I2C / serial
    def send(self, output: ControlOutput): ...  # map -1…+1 to PWM, write to servo
    def close(self): ...
```

Then in `inference.py`, add:

```python
from src.hardware import HardwareDriver
hw = HardwareDriver()
# ... inside the loop:
hw.send(output)
```

The `ControlOutput` dataclass is already structured for direct consumption —
no other changes needed.

---

## 12. Troubleshooting

### Camera not found

```
CameraError: Cannot open camera device '0'
```

Check available devices:

```bash
ls /dev/video*
```

Try a different index:

```bash
python inference.py --device 1
```

### Model weights not found

```
FileNotFoundError: Weights file not found: weights/best.pt
```

Either train first (`python train.py`) or leave `weights: ""` in
`config/model.yaml` to use the Ultralytics pretrained backbone.

### Roboflow API key missing

```
EnvironmentError: Roboflow API key not found.
```

```bash
export ROBOFLOW_API_KEY="rf_xxxx"
# or create .env in project root with ROBOFLOW_API_KEY=rf_xxxx
```

### Slow inference on Pi

- Use `yolov8n` (nano) model.
- Reduce `img_size` to 320 in `config/model.yaml`.
- Run `python inference.py --headless` to avoid X11 rendering overhead.
- Ensure the Pi is running in 64-bit mode: `uname -m` should return `aarch64`.

### Display window doesn't open over SSH

Use `--headless` flag or set up X forwarding:

```bash
ssh -X user@raspberrypi
python inference.py   # window will appear on your local machine
```
