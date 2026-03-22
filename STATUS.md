# Project Status

_Last updated: March 2026_

---

## What This Project Does

An autonomous apple-detection robot running on a Raspberry Pi 5. The Pi uses a USB endoscope camera to capture live video, runs a YOLOv8 model to detect apples in each frame, and outputs steering/drive/tilt commands to guide the robot toward whichever apple it finds.

---

## Current Status

### Done

| Area | Detail |
|---|---|
| **Camera** | DFRobot FIT0819 USB endoscope connected on `/dev/video0`, confirmed working with OpenCV (MJPEG mode, 640×480 @ 25 fps) |
| **Inference pipeline** | Runs headless on the Pi: camera → YOLO detect → tracker → controller → terminal output |
| **Dataset** | 697 labelled apple images (489 train / 178 val / 30 test), classes: `apple` + `damaged_apple`, stored at `data/images/` |
| **Dataset config** | `config/dataset.yaml` set to `source: local`, pointing at `data/images/` |
| **Colab notebook** | `train_colab.ipynb` ready to upload — handles install, dataset upload, training, and `best.pt` download automatically |
| **Training** | In progress on Google Colab (T4 GPU), 50 epochs, YOLOv8n, image size 640 |
| **Controller** | Proportional control logic implemented; outputs `steering_servo`, `drive_motor`, `camera_tilt` in range −1.0 … +1.0 |
| **Hardware config** | `config/hardware.yaml` present with stub interface — no physical hardware wired yet |

### In Progress

- **Model training** — currently running on Google Colab. Once complete, `best.pt` needs to be copied to `weights/best.pt` on the Pi.

### Not Started Yet

- Wiring up servos/motor to the Pi (hardware interface is currently set to `stub`)
- Selecting and configuring a hardware interface (`pca9685` / `gpio` / `serial`) in `config/hardware.yaml`
- Field testing with the trained model

---

## How to Run Inference (Pi)

```bash
cd ~/yolo-project
source .venv/bin/activate
python inference.py --device 0 --headless
```

> Currently uses the generic pretrained YOLOv8n backbone. Once `best.pt` is in place this will use the trained apple model.

---

## After Training Completes

1. Copy `best.pt` from your PC to the Pi:
   ```bash
   scp best.pt subs@10.10.189.241:/home/subs/yolo-project/weights/best.pt
   ```

2. Edit `config/model.yaml` — set:
   ```yaml
   weights: weights/best.pt
   ```

3. Run inference as normal — the detector will load the trained weights automatically.

---

## Project Structure

```
yolo-project/
├── inference.py          # Main loop: camera → detect → track → control
├── train.py              # Training script (run on GPU machine, not Pi)
├── train_colab.ipynb     # Self-contained Google Colab training notebook
├── dataset.zip           # Zipped dataset ready for Colab upload
│
├── config/
│   ├── model.yaml        # Architecture, weights path, confidence thresholds
│   ├── dataset.yaml      # Dataset source (currently: local @ data/images)
│   └── hardware.yaml     # Servo/motor pins and PWM ranges (interface: stub)
│
├── src/
│   ├── camera.py         # USB camera wrapper (OpenCV + MJPEG)
│   ├── detector.py       # YOLOv8 inference wrapper
│   ├── tracker.py        # Picks best apple, computes frame-centre error
│   ├── controller.py     # Proportional controller → servo/motor commands
│   ├── display.py        # Optional OpenCV window (headless-safe)
│   └── dataset.py        # Dataset resolver (Roboflow or local)
│
├── data/images/          # Training dataset (489 train / 178 val / 30 test)
├── weights/              # Trained model weights go here (best.pt)
└── runs/                 # Training outputs (created by train.py)
```

---

## Key Settings

| File | Setting | Current Value |
|---|---|---|
| `config/model.yaml` | `architecture` | `yolov8n` |
| `config/model.yaml` | `weights` | _(empty — awaiting best.pt)_ |
| `config/model.yaml` | `confidence` | `0.50` |
| `config/dataset.yaml` | `source` | `local` |
| `config/dataset.yaml` | `local_path` | `data/images` |
| `config/hardware.yaml` | `interface` | `stub` _(no hardware yet)_ |
