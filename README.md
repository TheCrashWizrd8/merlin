# YOLO Apple RC Sub Detection System

Modular YOLO-based apple detection for an RC submarine running on a Raspberry Pi 5 (8 GB).  
The Pi uses one or two USB endoscope cameras, runs YOLO each frame (NCNN on the Pi), and produces a structured `ControlOutput` that drives steering, thruster, and camera tilt to keep an apple centred in view. With two cameras (centres 16 cm apart, 67° FOV) it also reports stereo range.

A separate **sub vehicle stack** adds an ESP32-S3 sensor/actuator hub, ballast control, leak/depth/IMU telemetry, an Xbox controller input path, and a web dashboard at `/sub/`.

---

## Quick Start

### 1. Install dependencies

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

### 2. Configure

Edit the YAML files in `config/`:

| File | Purpose |
|------|---------|
| `config/model.yaml` | YOLO architecture, `backend` (`ncnn` / `pytorch` / `openvino`), weights path, confidence, **model catalog** (Auto-mode picker on `/sub/`) |
| `config/dataset.yaml` | Dataset source (`local` or optional `roboflow`) |
| `config/hardware.yaml` | Control tuning, stereo cameras, ESP serial (sub), Xbox deadzone |
| `config/xbox_mapping.yaml` | Xbox button/stick → actuator bindings |
| `config/pins.yaml` | Pin/channel reference (documentation; not read at runtime yet) |

**Hardware:** The Pi talks to an **ESP32-S3** over **GPIO UART** (`/dev/serial0`) or USB (`/dev/ttyACM0`) for the full sub stack (actuators, ballast, sensors, telemetry). See **`docs/ESP32_SERIAL.md`** and **`esp32/README.md`**.

### 3. Dataset & training (Google Colab — not on Pi)

Train on **[Google Colab](https://colab.research.google.com/)** (free GPU) so you don't need a local GPU.

1. Prepare a YOLO-format dataset with `data.yaml` at the root.
2. Upload the project and dataset to Colab (or clone from Git / mount Google Drive).
3. Set `config/dataset.yaml`:

```yaml
source: local
local_path: "data/images"   # folder containing data.yaml
```

4. Install deps and train:

```python
!pip install ultralytics pyyaml roboflow python-dotenv
!python train.py
```

5. Download **`weights/best.pt`** to the Pi, export NCNN (`python scripts/export_model.py --format ncnn`), and confirm `active_model: detect` / `weights:` in **`config/model.yaml`**.

Full Colab steps are in **`docs/GUIDE.md`** §5–6. A ready-made notebook is at **`train_colab.ipynb`**.

### 4. Run YOLO inference on the Pi

With **`interface: sub`** in `config/hardware.yaml` (default), this starts YOLO + ESP bridge + sub dashboard automatically:

```bash
source .venv/bin/activate
python inference.py --web --timing
```

| URL | Content |
|-----|---------|
| `http://<pi-ip>:8080/` | YOLO MJPEG stream with HUD |
| `http://<pi-ip>:8080/sub/` | Sub telemetry, actuators, ballast, serial monitor |

Inference sets sub control mode to **auto** and uses **layered motion** (fins → aft steer → thruster + ballast height). See **`docs/GUIDE.md`** §14–15.

Local OpenCV window (stub hardware only):

```bash
python inference.py
```

Press **q** to quit.

Headless (SSH, no browser):

```bash
python inference.py --headless
```

### 5. Web stream (YOLO camera feed)

Included automatically when you run inference with `interface: sub` or pass `--web`:

```bash
python inference.py --web
```

Open **http://\<pi-ip\>:8080** — live MJPEG at `/video_feed`. Sub dashboard at **`/sub/`** uses the same MJPEG feed plus an SSE live data stream (no polling).

Use **`--timing`** to print capture/YOLO/draw breakdown every 30 frames. Terminal output is quiet by default when the web/sub server is active (`--quiet` to force).

### 6. Sub dashboard (ESP telemetry + actuators + Xbox)

**Without YOLO** (diagnostics / bench testing):

```bash
python sub_server.py
```

**With YOLO auto control** (layered actuators + ballast from apple tracking):

```bash
python inference.py --web    # when interface: sub in config (recommended)
# or explicitly:
python inference.py --web --sub
```

Open **http://\<pi-ip\>:8080/sub/** — telemetry, ballast, actuators, pin tests, serial monitor.

**Control:** bench default is **Manual** (sliders at zero). During inference the server runs **auto**; click **Auto (YOLO)** on the dashboard if your browser restored Manual from a previous session.

**Multiple YOLO models:** In **Auto (YOLO)** mode the dashboard shows a second row of model buttons (detect, seg, …). Models are defined in **`config/model.yaml`** under `models:`; only entries with weights + NCNN export on disk are clickable. Today only **`detect`** (`weights/best.pt`) is deployed — add new `.pt` files and catalog entries as you train. Full details: **`docs/GUIDE.md`** §11.

**Xbox controller (Bluetooth):** pair once at the OS level, verify with `python -m src.xbox_controller`. Bindings in **`config/xbox_mapping.yaml`**; stick drift tuning in **`config/hardware.yaml`** (`xbox.deadzone`). Full steps in **`docs/GUIDE.md`** (§ Connecting the Xbox controller).

Simulated telemetry (no ESP hardware):

```bash
python scripts/test_telemetry.py
```

---

## Multiple YOLO models (catalog & dashboard picker)

Inference can run **several trained models** and switch between them **without restarting** the Pi process. Configuration is in **`config/model.yaml`**.

### Catalog (current)

| ID | Dashboard label | Weights | Task | On Pi today |
|----|-----------------|---------|------|-------------|
| `detect` | Detect | `weights/best.pt` | detect | Yes — YOLOv8n + `best_ncnn_model/` |
| `seg` | Seg (future) | `weights/best_seg.pt` | segment | No — add when trained |
| `seg_fast` | Seg 320 (future) | `weights/best_seg_320.pt` | segment | No — optional faster seg variant |

`active_model: detect` controls which entry loads on startup.

### Switch from the dashboard

1. `python inference.py --web --timing`
2. Open **`/sub/`** → **Auto (YOLO)**
3. A **YOLO model** button row appears — click a model to reload (few seconds)

Unavailable models are greyed out (missing `.pt` or NCNN export). Your choice persists in browser `sessionStorage`.

### Switch from the API

```bash
curl http://<pi-ip>:8080/sub/api/models
curl -X POST http://<pi-ip>:8080/sub/api/models/select \
  -H 'Content-Type: application/json' \
  -d '{"id":"detect"}'
```

### Add a new model

1. Put `weights/your_model.pt` on the Pi  
2. `python scripts/export_model.py --weights weights/your_model.pt --format ncnn`  
3. Add an entry under `models:` in `config/model.yaml` (`task: segment` for YOLOv8-seg)  
4. Restart inference once (to pick up new YAML keys) or select from the dashboard if already listed  

Each entry can set its own **`track_label`** (class name to follow). Segmentation models use mask centroids for aim and stereo when masks are available.

See **`docs/GUIDE.md`** §11 for architecture notes (`src/model_runtime.py`, stale-export guard, SSE `models` payload).

---

## Serial link to ESP32

The Pi communicates with the ESP32 over GPIO UART or USB using the **`sub_rc`** firmware:

| Link | Port | Protocol | Used by | Firmware |
|------|------|----------|---------|----------|
| **Sub vehicle** | `/dev/serial0` or `/dev/ttyACM0` | `S2 … F … X …`, `B …`, `TEL …` | `esp_bridge.py` (single owner) via `SubBridgeOutput` | `esp32/sub_rc/` |

One serial client only — `SubBridgeOutput` writes to `sub_state`; `esp_bridge` sends `S2`/`B`. No duplicate legacy S/D/T port access.

Flash **`esp32/sub_rc/`** and wire Pi GPIO14 TX / GPIO15 RX to the ESP32 (or use USB serial).

---

## Project Structure

```
yolo-project/
├── README.md
├── STATUS.md                   # Current project status
├── requirements.txt
├── inference.py                # YOLO inference loop (+ --web, --sub)
├── sub_server.py               # Sub dashboard without YOLO
├── train.py                    # Training (GPU / Colab)
├── train_colab.ipynb           # Self-contained Colab notebook
├── config/
│   ├── model.yaml
│   ├── dataset.yaml
│   ├── hardware.yaml           # Control tuning + stereo cameras + serial
│   └── pins.yaml               # Pin reference (ESP32, PCA9685, L298N)
├── data/                       # YOLO datasets
├── weights/                    # best.pt + best_ncnn_model/ (detect today; add more .pt later)
├── src/
│   ├── detector.py             # YOLO wrapper (pytorch / ncnn / openvino)
│   ├── model_runtime.py        # Model catalog + hot-swap for Auto mode
│   ├── stereo.py               # Dual-cam triangulation → range_m
│   ├── tracker.py              # Apple centroid, bbox, normalised error
│   ├── controller.py           # ControlOutput + size-based drive + telemetry safety
│   ├── telemetry_context.py    # ESP snapshot for controller + sub motion
│   ├── hardware.py             # stub / SubBridgeOutput (YOLO → sub_state)
│   ├── camera.py               # USB camera capture (MJPEG)
│   ├── display.py              # Annotated live view + HUD FPS
│   ├── control_source.py       # Manual vs auto for YOLO S/D/T
│   ├── web_stream.py           # Flask MJPEG + /snapshot + frame buffer
│   ├── dataset.py              # Resolve data.yaml (local or Roboflow)
│   ├── esp_bridge.py           # Pi ↔ ESP32 serial (telemetry + sub cmds)
│   ├── sub_state.py            # Thread-safe sub telemetry/control state + SSE notifications
│   ├── sub_control.py            # yolo_to_sub_motion() entry point
│   ├── sub_motion.py           # Layered auto: fins, steer, thruster, ballast
│   ├── sub_web.py              # /sub/* Flask routes + SSE stream
│   ├── sub_dashboard.html      # Sub web UI (EventSource + MJPEG)
│   ├── xbox_controller.py      # Gamepad reader (pygame thread)
│   ├── xbox_mapping.py         # config/xbox_mapping.yaml → actuators
│   └── serial_util.py          # Serial port helpers
├── esp32/
│   ├── sub_rc/                 # Primary sub firmware (UART + sensors)
│   ├── sub_hw_test/            # Hardware bring-up test
│   └── upload_from_pi.sh       # Compile + flash from Pi
├── scripts/
│   ├── test_telemetry.py       # Simulated ESP telemetry for UI dev
│   ├── simulate_esp_serial.py  # Fake ESP on a PTY
│   ├── probe_esp_uart.py       # Probe GPIO UART (PING + listen)
│   ├── export_model.py         # Export .pt → NCNN / OpenVINO
│   └── check_camera.sh         # Camera sanity check
└── docs/
    ├── GUIDE.md                # Full system guide
    ├── ESP32_SERIAL.md         # Serial protocol (sub)
    ├── PI_ESP32_COMPATIBILITY.md
    └── sub_endpoints_plan.md   # Sub API reference (implemented)
```

---

## ControlOutput (YOLO path)

Every inference frame produces a `ControlOutput`:

| Field | Range | Description |
|-------|-------|-------------|
| `steering_servo` | -1.0 → +1.0 | Left (-) to right (+) |
| `drive_motor` | -1.0 → +1.0 | Reverse (-) to forward (+) |
| `camera_tilt_servo` | -1.0 → +1.0 | Down (-) to up (+) |
| `target_x`, `target_y` | px | Apple centre |
| `error_x`, `error_y` | -1.0 → +1.0 | Normalised offset from frame centre |
| `confidence` | 0–1 | YOLO detection confidence |
| `apple_detected` | bool | Whether an apple was found |
| `bbox_x1`…`bbox_y2` | px | Tracked apple bounding box |
| `size_ratio_raw` / `size_ratio_filtered` | 0–1 | `bbox_area/frame_area` |
| `range_m` | metres or `None` | Stereo distance from camera midpoint to apple |
| `timestamp` | float | `time.time()` |

Size-based forward drive is configured in `config/hardware.yaml`. See **`docs/GUIDE.md`** for the full system guide.

---

## Sub actuators (serial path)

When running inference with `interface: sub` or `sub_server.py`, the ESP bridge sends:

```
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>
B <fore> <aft>
```

Control modes: **`manual`** (bench default), **`xbox`** (gamepad), **`auto`** (inference — layered motion in `sub_motion.py`).

**Auto mode mapping** (not a simple S/D/T mirror):

1. **Fins** — gyro roll/pitch leveling  
2. **Aft steer Y/Z** — camera errors (scaled while tilted)  
3. **Thruster** — forward drive, gated by attitude + alignment  
4. **Ballast** — fill when apple below centre, drain when above  

See **`docs/GUIDE.md`** §15 for full detail and `config/hardware.yaml` `sub_motion:` tuning.

---

## Related docs

| Doc | Contents |
|-----|----------|
| **`docs/GUIDE.md`** | Installation, training, inference, hardware, sub system, troubleshooting |
| **`docs/ESP32_SERIAL.md`** | Pi ↔ ESP32 serial protocols |
| **`docs/PI_ESP32_COMPATIBILITY.md`** | Pre-flight compatibility checklist |
| **`esp32/README.md`** | Firmware build, upload, pinout |
| **`STATUS.md`** | What's done, in progress, and not started |
| **`config/pins.yaml`** | Authoritative pin/channel map |
