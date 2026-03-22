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
| `config/dataset.yaml` | Roboflow project details (swap for prod) |
| `config/hardware.yaml` | Servo/motor interface (stub, pca9685, or serial to ESP32) |

**Hardware (Pi + ESP32 + L298N):** The Pi sends control values over USB serial to an **ESP32-S3**, which drives two **Hitec HS-646WP** servos and a **DC motor** via an **L298N**. Set `interface: serial` in `config/hardware.yaml`. Firmware: **`esp32/apple_car_rc/`** (Arduino sketch). Protocol and wiring: **`docs/ESP32_SERIAL.md`** and **`esp32/README.md`**.

Set your Roboflow API key as an environment variable (never hard-code it):

```bash
export ROBOFLOW_API_KEY="your_key_here"
```

Or create a `.env` file in the project root:

```
ROBOFLOW_API_KEY=your_key_here
```

### 3. Download dataset & train (GPU / Colab — not on Pi)

```bash
python train.py
```

This downloads the dataset from Roboflow, trains the model, and saves the best weights to  
`weights/best.pt`.

### 4. Run inference on the Pi

```bash
python inference.py
```

Opens the USB camera, runs YOLO detection each frame, and prints a live `ControlOutput`  
table to the terminal. A camera view window shows bounding boxes, the frame crosshair,  
and the error vector.

Press **q** to quit.

### 5. View stream in a browser (optional)

Run inference with the web server and open the stream in any device on your network:

```bash
python inference.py --web
```

Then open **http://\<pi-ip\>:8080** in a browser (e.g. `http://192.168.1.10:8080`).  
The page shows the live camera feed with bounding boxes and HUD.

**Over Tailscale:**  
The server binds to all interfaces (`0.0.0.0`), so it’s reachable on the Pi’s Tailscale IP. When you run with `--web`, the Tailscale URL is printed if `tailscale` is installed (e.g. `http://100.x.x.x:8080`). Open that URL from any device on your tailnet. If your firewall blocks the port, allow it (e.g. `sudo ufw allow 8080` then `sudo ufw reload`).

**If the page doesn’t load:**  
- Try from the Pi first: `http://localhost:8080`.  
- Test the server without the camera: `python scripts/test_web_server.py`, then open `http://localhost:8080`.  
- If port 8080 is in use, use another: `python inference.py --web --web-port 9090`.

---

## Project Structure

```
yolo-project/
├── README.md
├── requirements.txt
├── .env                        # (create locally, never commit)
├── config/
│   ├── model.yaml              # YOLO version, weights, thresholds
│   ├── dataset.yaml            # Dataset source — swap here for prod
│   └── hardware.yaml           # Servo/motor pin assignments and PWM ranges
├── data/                       # Downloaded datasets
├── weights/                    # Trained .pt weights
├── src/
│   ├── __init__.py
│   ├── detector.py             # YOLO inference wrapper
│   ├── tracker.py              # Apple centroid + normalised error
│   ├── controller.py           # ControlOutput dataclass + formatter
│   ├── hardware.py             # Output to stub / PCA9685 / serial (ESP32)
│   ├── camera.py               # USB camera capture
│   ├── display.py              # Annotated live view
│   └── dataset.py              # Roboflow dataset download helper
├── train.py                    # Training pipeline (GPU/Colab)
├── inference.py                # Main inference loop (Pi5)
└── docs/
    ├── GUIDE.md                # Full system guide
    └── ESP32_SERIAL.md         # Pi → ESP32 serial protocol (servos + L298N)
```

---

## Swapping the Dataset (prod)

1. Open `config/dataset.yaml`
2. Change `source: roboflow` → `source: local`
3. Set `local_path` to the directory containing your `data.yaml`
4. Run `python train.py` (or point `weights` in `model.yaml` to pre-trained weights)

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
| `timestamp` | float | `time.time()` |

See `docs/GUIDE.md` for the full system guide.
