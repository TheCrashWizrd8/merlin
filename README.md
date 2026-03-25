# YOLO Apple RC Car Detection System

Modular YOLO-based apple detection for an RC car running on a Raspberry Pi 5 (8 GB).  
The system detects apples via a USB camera, calculates their position relative to the frame  
centre, and produces a structured `ControlOutput` that drives steering, throttle, and camera  
tilt to keep the apple centred.

---

## Quick Start

### 1. Install dependencies

```bash
pip install -r requirements.txt
```

### 2. Configure

Copy and edit the config files in `config/`:

| File | Purpose |
|------|---------|
| `config/model.yaml` | YOLO architecture, weights path, confidence |
| `config/dataset.yaml` | Dataset source (`local` or optional `roboflow`) |
| `config/hardware.yaml` | Servo/motor interface (stub, pca9685, or serial to ESP32) |

**Hardware (Pi + ESP32 + L298N):** The Pi sends control values over USB serial to an **ESP32-S3**, which drives two **Hitec HS-646WP** servos and a **DC motor** via an **L298N**. Set `interface: serial` in `config/hardware.yaml`. Firmware: **`esp32/apple_car_rc/`** (Arduino sketch). Protocol and wiring: **`docs/ESP32_SERIAL.md`** and **`esp32/README.md`**.

### 3. Dataset & training (**Google Colab** — not on Pi)

**Recommended:** train on **[Google Colab](https://colab.research.google.com/)** (free GPU tier) so you don’t need a local GPU.

1. **Prepare** a YOLO-format dataset with a `data.yaml` at the root (train/val images and labels).
2. In Colab, **upload** the project (or clone from Git) and your dataset — e.g. zip the dataset, upload it, unzip under `data/` (or mount **Google Drive** and point to a folder there).
3. Set **`config/dataset.yaml`**:

```yaml
source: local
local_path: "data/your-dataset-folder"   # must contain data.yaml
```

4. Install deps and run training:

```python
!pip install ultralytics pyyaml roboflow python-dotenv
# optional: %cd to your project root if you cloned/unzipped the repo
!python train.py
```

5. **Download** **`weights/best.pt`** from Colab (Files sidebar) to your Pi, then set `weights:` in **`config/model.yaml`**.

> **Optional:** `dataset.yaml` can use `source: roboflow` with `ROBOFLOW_API_KEY` if you still use Roboflow for exports.

Full Colab steps (Drive, zips, troubleshooting) are in **`docs/GUIDE.md`** §5–6.

### 4. Run inference on the Pi

```bash
python inference.py
```

Opens the USB camera, runs YOLO each frame, and prints a live `ControlOutput` table. The window shows **real bounding boxes**, crosshair, error vector, and HUD.

Press **q** to quit.

### 5. View stream in a browser (optional)

```bash
python inference.py --web
```

Open **http://\<pi-ip\>:8080** in a browser (default port; use `--web-port` to change).  
Live feed + **http://\<pi-ip\>:8080/api/control** for manual/auto mode (see `src/web_stream.py`).

**Over Tailscale:**  
The server binds to all interfaces (`0.0.0.0`), so it’s reachable on the Pi’s Tailscale IP. When you run with `--web`, the Tailscale URL is printed if `tailscale` is installed (e.g. `http://100.x.x.x:8080`). If your firewall blocks the port, allow it (e.g. `sudo ufw allow 8080` then `sudo ufw reload`).

**If the page doesn’t load:**  
- Try from the Pi first: `http://localhost:8080`.  
- Test without the camera: `python scripts/test_web_server.py`, then open `http://localhost:8080`.  
- If port 8080 is in use: `python inference.py --web --web-port 9090`.

---

## Project Structure

```
yolo-project/
├── README.md
├── requirements.txt
├── .env                        # (create locally; optional for legacy Roboflow)
├── config/
│   ├── model.yaml              # YOLO version, weights, thresholds
│   ├── dataset.yaml            # Dataset source — local (e.g. Colab/Drive) or roboflow
│   └── hardware.yaml         # Servo/motor, serial, size-based drive
├── data/                       # Local datasets (unzipped on Colab or copied to Pi)
├── weights/                    # Trained .pt weights
├── src/
│   ├── __init__.py
│   ├── detector.py             # YOLO inference wrapper
│   ├── tracker.py              # Apple centroid, bbox, normalised error
│   ├── controller.py           # ControlOutput + size-based drive
│   ├── hardware.py             # stub / PCA9685 / serial (ESP32)
│   ├── camera.py               # USB camera capture
│   ├── display.py              # Annotated live view
│   ├── control_source.py       # Manual vs auto (web / RC)
│   ├── web_stream.py           # Flask MJPEG + API when --web
│   └── dataset.py              # Resolve data.yaml (local or Roboflow)
├── train.py                    # Training pipeline (GPU / Google Colab)
├── inference.py                # Main inference loop (Pi5)
└── docs/
    ├── GUIDE.md                # Full system guide
    ├── ESP32_SERIAL.md         # Pi → ESP32 serial protocol
    └── PI_ESP32_COMPATIBILITY.md
```

---

## Swapping the Dataset

1. Place a YOLO dataset where it contains **`data.yaml`** (e.g. unzip on **Colab** or copy to your PC).
2. Set `source: local` and `local_path` in `config/dataset.yaml`.
3. Run `python train.py` (e.g. in Colab) or point `weights` in `model.yaml` to existing weights.

---

## ControlOutput

Every inference frame produces a `ControlOutput` with:

| Field | Range | Description |
|-------|-------|-------------|
| `steering_servo` | -1.0 → +1.0 | Left (-) to right (+) |
| `drive_motor` | -1.0 → +1.0 | Reverse (-) to forward (+) |
| `camera_tilt_servo` | -1.0 → +1.0 | Down (-) to up (+) |
| `target_x` | px | Apple centre x |
| `target_y` | px | Apple centre y |
| `error_x` | -1.0 → +1.0 | Normalised horizontal error |
| `error_y` | -1.0 → +1.0 | Normalised vertical error |
| `confidence` | 0–1 | YOLO detection confidence |
| `apple_detected` | bool | Whether an apple was found |
| `bbox_x1`…`bbox_y2` | px | Tracked apple bounding box |
| `size_ratio_raw` / `size_ratio_filtered` | 0–1 | `bbox_area/frame_area` (instant / smoothed) |
| `timestamp` | float | `time.time()` |

Size-based forward drive is configured in `config/hardware.yaml` (`use_size_for_drive`, `size_drive_far` / `size_drive_close`, `size_min_ratio` / `size_max_ratio`). See comments in that file.

See **`docs/GUIDE.md`** for the full system guide (Google Colab training, inference flags, hardware, troubleshooting).
