# YOLO Apple RC Sub — System Guide

_Last updated: 24 September 2026 — entry point `~/sub`, Hailo inference service, pilot HUD, linked flap, Xbox emergency stop._

---

## Table of contents

1. [System overview](#1-system-overview)
2. [Hardware](#2-hardware)
3. [Software architecture](#3-software-architecture)
4. [Installation](#4-installation)
5. [Training (Colab / GPU)](#5-training-colab--gpu)
6. [Running the stack](#6-running-the-stack)
7. [Cameras & stereo range](#7-cameras--stereo-range)
8. [YOLO models & backends](#8-yolo-models--backends)
9. [Control behaviour](#9-control-behaviour)
10. [Sub motion (auto mode)](#10-sub-motion-auto-mode)
11. [Control modes & Xbox](#11-control-modes--xbox)
12. [Dashboard (`/sub/`)](#12-dashboard-sub)
13. [ESP32 & serial](#13-esp32--serial)
14. [GPS](#14-gps)
15. [HTTP API reference](#15-http-api-reference)
16. [Configuration reference](#16-configuration-reference)
17. [Scripts & tests](#17-scripts--tests)
18. [Troubleshooting](#18-troubleshooting)

### Quick commands

| Goal | Command |
|------|---------|
| Start everything (YOLO off) | `~/sub` |
| Start with Hailo YOLO | `~/sub --yolo --backend hailo --timing` |
| Dashboard | `http://<pi-ip>:8080/sub/` |
| Switch model (API) | `POST /sub/api/models/select` `{"id":"detect","backend":"hailo"}` |
| Test gamepad | `python -m src.xbox_controller` |
| Check cameras | `bash scripts/check_camera.sh` |
| Flash ESP32 | `bash esp32/upload_from_pi.sh` |
| Fake telemetry | `python scripts/test_telemetry.py` |

---

## 1. System overview

The Pi runs a **single process** (`~/sub` → `run.py` → `src/app.py`) that hosts:

- Flask web server (MJPEG + dashboard)
- ESP32 serial bridge (telemetry in, actuator commands out)
- Xbox gamepad polling (optional)
- USB GPS auto-scan (optional)
- **Inference service** — YOLO starts/stops from the dashboard without restarting the process

```
┌─────────────┐     ┌──────────────┐     ┌─────────────┐
│ USB cameras │────►│ YOLO + track │────►│ Controller  │
│ FOV + stereo│     │ (inference   │     │ ControlOutput│
└─────────────┘     │  service)    │     └──────┬──────┘
                    └──────────────┘            │
                                                ▼
                    ┌──────────────────────────────────┐
                    │ sub_motion.plan_sub_motion()      │
                    │ fins → aft steer → thrust/ballast │
                    └──────────────────┬───────────────┘
                                       ▼
                    ┌──────────────────────────────────┐
                    │ sub_state → esp_bridge → ESP32   │
                    │ S2 actuators + B ballast + TEL    │
                    └──────────────────────────────────┘
```

### Goal behaviour

| Situation | Response |
|-----------|----------|
| Apple off-centre horizontally | Aft steer Y tracks target |
| Apple off-centre vertically | Aft steer Z + ballast trim (fill/drain) |
| Sub rolled/pitched | Fins level body; thrust/steer gated until attitude OK |
| Stereo range available | Proximity-based drive scaling |
| Leak detected | Drive forced to zero |
| Low battery | Drive scaled down |
| Brief detection dropout | Last motion held (configurable frames) |
| Xbox B pressed | YOLO stops, all actuators zeroed |

---

## 2. Hardware

### Typical stack

| Component | Role |
|-----------|------|
| Raspberry Pi 5 (8 GB) | YOLO, control, web dashboard |
| Hailo-8L HAT | Primary inference accelerator |
| USB endoscope cameras | FOV (pilot view) + stereo pair (16 cm baseline, 67° FOV) |
| ESP32-S3 | Actuators, ballast, ADC, IMU, leak sensors |
| PCA9685 (I2C on ESP) | Aft steer + fin servos |
| L298N | Thruster |
| Ballast motors + pots | Fore/aft tanks with level feedback |
| Xbox controller | Manual / override input (USB or Bluetooth) |
| u-blox USB GPS | Optional position track on dashboard |

### Pi ↔ ESP32 serial

| Connection | Pi | ESP32 | Device |
|------------|-----|-------|--------|
| USB (default) | USB cable | USB CDC | `/dev/ttyACM0` or `/dev/serial/by-id/...` |
| GPIO UART | GPIO14 TX → ESP RX | GPIO15 RX ← ESP TX | `/dev/serial0` |

Set `sub_serial.port` in `config/hardware.yaml`. The bridge auto-detects Espressif devices under `/dev/serial/by-id/` when present.

### Interface modes (`config/hardware.yaml`)

| `interface` | Behaviour |
|-------------|-----------|
| `stub` | Log commands only — no ESP output |
| `sub` | Production — YOLO → `sub_state` → `esp_bridge` |

---

## 3. Software architecture

### Core modules

| Module | Role |
|--------|------|
| `src/app.py` | CLI entry, starts ESP/GPS/Xbox/inference service, Flask server |
| `src/inference_service.py` | YOLO lifecycle: start/stop/switch model; camera rig; control loop |
| `src/model_runtime.py` | Model catalog discovery, hot-swap, backend management |
| `src/detector.py` | YOLO wrapper (pytorch / ncnn / openvino / hailo) |
| `src/hailo_runtime.py` | HailoRT HEF loading and inference |
| `src/tracker.py` | Target selection, normalised errors, bbox/mask centroid |
| `src/stereo.py` | Triangulation from left/right detections → `range_m` |
| `src/controller.py` | Track → `ControlOutput`; telemetry safety; size/range drive |
| `src/sub_motion.py` | Layered auto planner + linked flap coupling |
| `src/hardware.py` | `SubBridgeOutput` — YOLO output → `sub_state` |
| `src/sub_state.py` | Thread-safe telemetry, control modes, SSE notifications |
| `src/esp_bridge.py` | Serial RX/TX (~20 Hz), parses `TEL`, sends `S2`/`B` |
| `src/sub_web.py` | `/sub/*` Flask routes + SSE stream |
| `src/sub_dashboard.html` | Pilot HUD + diagnostics UI |
| `src/web_stream.py` | MJPEG `/video_feed`, JPEG `/snapshot/<channel>` |
| `src/xbox_controller.py` | Gamepad poll thread, hot-plug, emergency stop |
| `src/xbox_mapping.py` | YAML → actuators; linked flap; override detection |
| `src/xbox_stick_filter.py` | EMA + hold filter for Bluetooth jitter |
| `src/gps_reader.py` | USB GPS auto-scan and track |
| `src/vision_state.py` | Detection overlay state for dashboard |
| `src/camera.py` | V4L capture, MJPEG preferred, reconnect |

### Data flow (one inference frame)

```
cameras.read() → detector.detect() → tracker.update()
    → stereo.triangulate() (if 2 cams)
    → controller.compute(track, telemetry, range_m)
    → hardware.apply(output)
        → sub_motion.plan_sub_motion()
        → sub_state.set_auto_actuators()
        → esp_bridge sends S2/B from recompute_effective()
```

### Actuator command format (ESP)

```
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>
B <fore_ballast> <aft_ballast>
```

All values are **−1.0 … +1.0**. See [`docs/ESP32_SERIAL.md`](ESP32_SERIAL.md) for full `TEL` telemetry format.

---

## 4. Installation

### Pi 5

```bash
sudo apt update && sudo apt upgrade -y
sudo apt install -y libopencv-dev python3-opencv v4l-utils

python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

**Hailo:** Install HailoRT per Hailo docs for Pi. Compile `.hef` on an x86 machine with Hailo Dataflow Compiler:

```bash
# On dev PC:
python scripts/export_model.py --weights weights/detect/best.pt --format hailo
# Copy hailo/ folder back to Pi
```

**NCNN fallback** (CPU):

```bash
pip install -r requirements-export.txt
python scripts/export_model.py --weights weights/detect/best.pt --format ncnn
```

Set `backend: ncnn` in `config/model.yaml` if not using Hailo.

> ARM64 PyTorch: if install fails, use CPU wheels:  
> `pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu`

### Training machine / Colab

```bash
pip install ultralytics pyyaml roboflow python-dotenv
```

Use [`train_colab.ipynb`](../train_colab.ipynb) or `python train.py` on a GPU machine.

---

## 5. Training (Colab / GPU)

1. Prepare YOLO-format dataset with `data.yaml` at the root.
2. Set `config/dataset.yaml`:

```yaml
source: local
local_path: "data/images"
```

3. Train:

```bash
python train.py
```

4. Copy `weights/<id>/best.pt` to the Pi.
5. Export for inference backends (see §8).
6. Set `active_model: <id>` in `config/model.yaml`.

Training hyperparameters (`epochs`, `batch_size`, etc.) live in `config/model.yaml`.

---

## 6. Running the stack

### Canonical command

```bash
~/sub
```

`~/sub` is a shell wrapper in the project (`yolo-project/sub`) that `cd`s to `~/yolo-project` and runs `.venv/bin/python run.py`. Install once:

```bash
ln -sf ~/yolo-project/sub ~/sub
```

Equivalent from inside the repo: `python run.py` (also `sub_server.py`, `inference.py`).

Override project path: `SUB_PROJECT=/path/to/yolo-project ~/sub`

### Common flags

```bash
~/sub --help

~/sub --yolo --backend hailo --model detect --timing
~/sub --serial-port /dev/ttyACM0
~/sub --no-xbox --no-gps
~/sub --no-camera          # dashboard only
~/sub --left-device /dev/video2 --right-device /dev/video4
~/sub --fov-width 1280 --fov-height 720
```

| Flag | Effect |
|------|--------|
| `--yolo` | Start inference immediately (else pick model on dashboard) |
| `--backend hailo\|ncnn\|pytorch\|openvino` | Override `config/model.yaml` backend |
| `--model <id>` | Catalog id to load with `--yolo` |
| `--timing` | Log inference loop FPS every ~30 s |
| `--no-sub` | Skip ESP, GPS, Xbox |
| `--no-esp` / `--no-xbox` / `--no-gps` | Skip individual peripherals |
| `--host 0.0.0.0 --port 8080` | Web bind (defaults) |

### What starts automatically

| Service | When |
|---------|------|
| Flask + dashboard | Always |
| ESP bridge | Unless `--no-esp` or `--no-sub` |
| GPS scan | If `gps.enabled: true` |
| Xbox polling | If `xbox.enabled: true` |
| Camera preview | Unless `--no-camera` |
| YOLO inference | Only with `--yolo` or dashboard model select |

### Shutdown

`Ctrl+C` — process exits cleanly. YOLO unloads; cameras may stay open until process exit.

---

## 7. Cameras & stereo range

### Three camera roles

| Role | Config key | Snapshot URL | Use |
|------|------------|--------------|-----|
| FOV / pilot | `fov_device` | `/snapshot/fov` | HUD background, `/video_feed` |
| Stereo left | `left_device` | `/snapshot/yolo_left` | YOLO + triangulation |
| Stereo right | `right_device` | `/snapshot/yolo_right` | YOLO + triangulation |

Use **stable by-path devices** so USB plug order does not swap left/right:

```bash
python scripts/camera_paths.py
```

Example from `config/hardware.yaml`:

```yaml
cameras:
  num_cameras: 2
  left_device: /dev/v4l/by-path/platform-xhci-hcd.0-usb-0:1:1.0-video-index0
  fov_device:  /dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0
  right_device: /dev/v4l/by-path/platform-xhci-hcd.1-usb-0:1:1.0-video-index0
  fov_width: 0    # 0 = auto max MJPEG
  fov_height: 0
  baseline_cm: 16.0
  fov_h_deg: 67.0
  fov_is_diagonal: true
  range_scale: 1.045
```

### Stereo triangulation

When both cameras detect the target, `src/stereo.py` triangulates bbox centres → `range_m`. Used for proximity drive scaling (`approach.use_stereo_range: true`).

Tune `range_scale` after a tape-measure check: `range_scale = tape_m / HUD_m`.

### Camera troubleshooting

```bash
bash scripts/check_camera.sh
v4l2-ctl --list-devices
```

Many UVC cameras expose **two** nodes per physical camera (capture + metadata). Open the **capture** node (usually even index).

---

## 8. YOLO models & backends

### Catalog layout

Each model lives in `weights/<id>/`:

```
weights/detect/
  model.yaml          # label, task, track_label, default_backend
  best.pt             # Ultralytics checkpoint
  hailo/*.hef         # Hailo package (preferred)
  best_ncnn_model/    # CPU fallback
```

See [`weights/README.md`](../weights/README.md) for full layout. Copy `weights/_template/` to add a new id.

`config/model.yaml` `models:` entries **overlay** sidecar metadata. Folders are auto-discovered.

### Backends

| Backend | Where it runs | Notes |
|---------|---------------|-------|
| `hailo` | Hailo-8L HAT | Intended production path; one HEF loaded at a time |
| `ncnn` | Pi CPU | Fast CPU fallback (~68 ms vs ~304 ms PyTorch on Pi 5) |
| `pytorch` | Pi CPU | Slowest; useful for debugging |
| `openvino` | Pi CPU | Optional |

Hailo-8L holds **one** HEF. Loading a second Hailo model parks the first on NCNN/CPU automatically.

### Switch models

**Dashboard:** Mode → **Auto (YOLO)** → **Model** dropdown (top-left HUD). Picking a model starts or switches inference.

**API:**

```bash
curl http://localhost:8080/sub/api/models
curl -X POST http://localhost:8080/sub/api/models/select \
  -H 'Content-Type: application/json' \
  -d '{"id":"gate","backend":"hailo"}'
curl -X POST http://localhost:8080/sub/api/models/stop
```

Switching is **live** — no process restart. Queued if a load is already in progress.

### Export

```bash
python scripts/export_model.py --weights weights/detect/best.pt --format hailo
python scripts/export_model.py --weights weights/detect/best.pt --format ncnn
bash scripts/compile_hailo_hef.sh   # if using DFC pipeline directly
```

### Segmentation models

Set `task: segment` in model sidecar. Tracker uses mask centroid when masks are available. `track_label` selects which class to follow.

---

## 9. Control behaviour

### ControlOutput fields

| Field | Range | Meaning |
|-------|-------|---------|
| `steering_servo` | −1…+1 | Horizontal aim → aft steer Y |
| `camera_tilt_servo` | −1…+1 | Vertical aim → aft steer Z |
| `drive_motor` | −1…+1 | Forward intent → thruster (gated in sub motion) |
| `error_x`, `error_y` | −1…+1 | Normalised offset from frame centre |
| `apple_detected` | bool | Target found this frame |
| `confidence` | 0…1 | Detection confidence |
| `range_m` | metres | Stereo range when available |

### Controller tuning (`config/hardware.yaml`)

Key knobs:

```yaml
gain_steer: 1.8
gain_tilt: 1.4
min_steer_command: 0.25    # sustain until centred
min_tilt_command: 0.18
min_drive_command: 0.55
hold_missed_frames: 12
smoothing_alpha: 0.30

approach:
  use_stereo_range: true
  range_far_m: 1.8
  range_near_m: 0.35
  stop_on_leak: true
  min_battery_v: 10.5
```

Profiles: `--control-profile stable|aggressive|config`.

---

## 10. Sub motion (auto mode)

`src/sub_motion.py` maps `ControlOutput` + gyro telemetry → sub actuators. Order:

1. **Fins** — gyro roll/pitch leveling (when linked flap off)
2. **Aft steer Y/Z** — camera errors, scaled while still tilted
3. **Thruster** — forward drive, gated by attitude + alignment
4. **Ballast** — fill when apple below centre, drain when above

### Linked flap (YOLO + Xbox)

When **linked flap** is ON (default):

- Fore fins **oppose the aft-steer axis angle** (same coupling as Xbox left stick)
- Right-stick fin input is replaced (`replace_stick: true`)
- YOLO uses the same logic via `sub_motion.linked_flap: true`

Toggle on Xbox: **D-pad left**. Dashboard badge: **Flap — LINKED** / **FREE**.

Config (`config/xbox_mapping.yaml`):

```yaml
modes:
  linked_flap:
    button: dpad_left
    toggle: true
    default_on: true
    gain: 1.0
    replace_stick: true
    angle_deadzone: 0.05
```

### Motion hold & smoothing

| Setting | Effect |
|---------|--------|
| `hold_missed_frames` | Keep last steer/thrust/ballast through brief detection gaps |
| `output_smoothing_alpha` | EMA on actuator outputs (0 = off) |

When apple is visible and centred, motion stops intentionally (no stale thrust).

### Ballast height trim

`error_y > 0` → apple below centre → fill (sink toward target).  
`error_y < 0` → apple above → drain (rise).

---

## 11. Control modes & Xbox

### Three modes (`sub_state.control_mode`)

| Mode | Actuator source | Set by |
|------|-----------------|--------|
| `manual` | Dashboard sliders | Default on bench; `POST /sub/api/control` |
| `xbox` | Mapped gamepad | Pad connect (from manual); mode dropdown |
| `auto` | YOLO + `sub_motion` | Model select; `--yolo`; inference start |

**Effective output** (what ESP receives) is computed in `sub_state.recompute_effective()`:

- `auto` + no override → `auto_actuators`
- `auto` + Xbox override → `xbox_actuators`
- `xbox` → `xbox_actuators`
- `manual` → `manual_actuators`

### Xbox layout (`config/xbox_mapping.yaml`)

| Input | Action |
|-------|--------|
| **Left stick** | Aft steer Y / Z |
| **Right stick** | Fins (when linked flap OFF) |
| **RT / LT** | Thruster forward / reverse |
| **RB / LB** | Ballast up / down (both tanks) |
| **D-pad up/down + RB/LB** | Fore / aft tank select |
| **D-pad left** | Toggle linked flap |
| **B** | **Emergency stop** — stop YOLO, zero all movement |

### Override during YOLO

With `xbox.override_auto: true` (default):

- YOLO keeps running and updating detections
- Deliberate pad input (post deadzone + stick filter) switches **effective** output to Xbox
- Override latched `override_hold_s` (0.4 s) after sticks centre
- Dashboard badge: **auto (Xbox override)**

Override uses **mapped actuators**, not raw stick magnitude — stick drift does not steal control.

### Emergency stop (B button)

Press **B** once:

1. Stops YOLO inference
2. Zeros all actuators and ballast commands
3. Switches to **Xbox** mode (pad control) or manual if no pad

Config:

```yaml
emergency_stop:
  enabled: true
  button: b
```

### Stick tuning

```yaml
xbox:
  deadzone: 0.18
  trigger_deadzone: 0.08
  smoothing_alpha: 0.35
  stick_hold_ms: 120
  release_alpha: 0.55
  override_actuator_threshold: 0.12
```

Verify: `python -m src.xbox_controller`

### Bluetooth pairing (one-time)

1. Hold **Sync** on controller until Xbox logo flashes rapidly.
2. Pair via `bluetoothctl` or desktop Bluetooth menu.
3. Verify `/dev/input/js0` appears.
4. Run `python -m src.xbox_controller` before starting the stack.

Disable ERTM if pairing drops:

```bash
echo 'options bluetooth disable_ertm=Y' | sudo tee /etc/modprobe.d/bluetooth-xbox.conf
sudo reboot
```

---

## 12. Dashboard (`/sub/`)

### Layout

**Pilot view (first screen — no scroll):**

| Area | Content |
|------|---------|
| Top-left | Mode dropdown + Model dropdown (Auto only) |
| Top-centre | Status badges (ESP, PCA9685, Xbox, flap, leaks, cams, mode) |
| Top-right | Battery, depth, accel bars |
| Left | Ballast faders (command) + level bars (telemetry) |
| Right | Thrust fader |
| Bottom-left | Aft steer crosshair |
| Bottom-centre | Artificial horizon (gyro) |
| Bottom-right | Fin L/R bars |

FOV camera fills the background via live JPEG (`/snapshot/fov`).

**Diagnostics (scroll down):**

- YOLO stereo feeds (lazy-loaded when scrolled into view)
- Model + backend selectors, Start/Stop
- Serial monitor, ESP test buttons, ballast calibration
- GPS track map, leak grid, pin reference

### Live data

- **SSE:** `GET /sub/api/stream` — pushes telemetry, control, models, serial on change (~10 Hz cap)
- **No polling** for main HUD — EventSource drives updates
- **Persistence:** mode, model, sliders saved in `sessionStorage` (survives refresh)

### Starting YOLO from UI

1. Set mode to **Auto (YOLO)**
2. Pick **Model** from dropdown (HUD) or diagnostics panel
3. Backend auto-selected from model sidecar (Hailo preferred)
4. Status line shows loading → running

Leaving Auto mode or pressing **B** on Xbox stops inference.

### Fader behaviour

- Ballast/thrust faders sync from server when mode is Auto or Xbox
- Manual mode: faders are local-only until pushed to server
- Custom vertical faders (pointer-driven, not rotated `<input range>`)

---

## 13. ESP32 & serial

### Firmware

Primary sketch: `esp32/sub_rc/sub_rc.ino`

```bash
bash esp32/upload_from_pi.sh scan    # find port
bash esp32/upload_from_pi.sh         # build + flash
```

Bring-up / isolation sketches: `esp32/subrcservoisolate/`, `esp32/test/`

### Bridge behaviour (`src/esp_bridge.py`)

- Reads `TEL …` telemetry lines → updates `sub_state`
- Sends `S2` + `B` at ~20 Hz from `recompute_effective()`
- Keepalive if unchanged for 2 s
- Stale detection if no telemetry for 5 s
- Serial log ring buffer exposed on dashboard

### Probe without full stack

```bash
python scripts/probe_esp_uart.py --port /dev/ttyACM0
python scripts/simulate_esp_serial.py   # fake ESP on PTY
```

---

## 14. GPS

USB GPS auto-scan when `gps.enabled: true`:

```yaml
gps:
  enabled: true
  scan_interval_s: 3.0
  track_max_points: 1000
  min_move_m: 5.0
  origin_settle_fixes: 12
```

**Note:** ESP32 and GPS may both appear as `/dev/ttyACM0` — bridge prefers Espressif by-id paths; GPS scan skips the ESP port.

Dashboard shows fix status, satellite count, and track map in diagnostics. Clear track: **Clear track** button or `POST /sub/api/gps/clear`.

---

## 15. HTTP API reference

Base: `http://<pi-ip>:8080`

### Stream & snapshots

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/stream` | GET (SSE) | Combined dashboard push (telemetry, control, models, serial) |
| `/snapshot/fov` | GET | FOV JPEG |
| `/snapshot/yolo_left` | GET | Left stereo JPEG |
| `/snapshot/yolo_right` | GET | Right stereo JPEG |
| `/video_feed` | GET | MJPEG stream |
| `/sub/api/vision/detections` | GET | Detection overlay JSON for FOV |

### Telemetry

| Route | Description |
|-------|-------------|
| `/sub/api/telemetry` | Full telemetry snapshot |
| `/sub/api/telemetry/battery` | Battery voltage |
| `/sub/api/telemetry/gyro` | IMU pitch/roll/yaw |
| `/sub/api/telemetry/depth` | Depth sensor |
| `/sub/api/telemetry/ballast` | Tank levels + ADC |
| `/sub/api/telemetry/leaks` | Leak zones |
| `/sub/api/telemetry/sonar` | Sonar |
| `/sub/api/telemetry/gps` | GPS fix + track |

### Control

| Route | Method | Body | Description |
|-------|--------|------|-------------|
| `/sub/api/control` | GET | — | Control snapshot |
| `/sub/api/control` | POST | `{"mode":"manual\|xbox\|auto"}` | Set control mode |
| `/sub/api/control/actuators` | POST | `{aftSteerY, thrusterX, finLeft, …}` | Manual actuators |
| `/sub/api/control/ballast` | POST | `{fore, aft}` or `{value, tank}` | Ballast command |
| `/sub/api/actuators` | GET | — | All actuator columns |

JSON keys accept camelCase or snake_case: `aftSteerY` / `aft_steer_y`, etc.

### Models

| Route | Method | Body | Description |
|-------|--------|------|-------------|
| `/sub/api/models` | GET | — | Catalog + running status |
| `/sub/api/models/select` | POST | `{"id":"detect","backend":"hailo"}` | Start/switch model |
| `/sub/api/models/stop` | POST | — | Stop inference |

### ESP / diagnostics

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/serial` | GET/POST | Serial log / send raw line |
| `/sub/api/pins` | GET | Expected + live pin map |
| `/sub/api/diagnostics` | GET | Diagnostics snapshot |
| `/sub/api/test` | POST | Send ESP test command |
| `/sub/api/test/run` | POST | Run all hardware tests |
| `/sub/api/ballast/calibrate` | POST | `{"tank":"fore","end":"top"}` |

---

## 16. Configuration reference

### `config/model.yaml`

| Key | Purpose |
|-----|---------|
| `architecture` | Ultralytics model string (ignored when `.pt` defines arch) |
| `task` | `detect` / `segment` / `auto` |
| `backend` | Default runtime: `hailo`, `ncnn`, `pytorch`, `openvino` |
| `weights` | Default weights path |
| `active_model` | Catalog id loaded on `--yolo` without `--model` |
| `models` | Optional overlays for catalog entries |
| `confidence`, `iou` | Detection thresholds |
| `img_size` | Input size (640 typical) |
| `ncnn_threads` | CPU thread count (3 recommended on Pi 5) |

### `config/hardware.yaml`

| Section | Purpose |
|---------|---------|
| `cameras` | Devices, stereo geometry, range tuning |
| `interface` | `stub` or `sub` |
| `sub_serial` | ESP port + baud |
| `gps` | USB GPS scan settings |
| `xbox` | Deadzone, smoothing, override |
| `gain_*`, `min_*_command` | Controller tuning |
| `approach` | Stereo range drive, leak/battery safety |
| `sub_motion` | Fin leveling, gating, ballast trim, linked flap, hold/smooth |

### `config/xbox_mapping.yaml`

| Section | Purpose |
|---------|---------|
| `sticks` | Left = aft steer, right = fins |
| `triggers` | RT/LT = thrust |
| `ballast` | RB/LB + d-pad tank select |
| `emergency_stop` | B button behaviour |
| `modes.linked_flap` | Toggle + gain + replace_stick |
| `leak` | Rumble on leak alarm |

### `config/pins.yaml`

Reference only — documents ESP GPIO, PCA9685 channels, L298N, ballast, leaks. Not read at runtime by Python (firmware + docs use it).

---

## 17. Scripts & tests

| Script | Purpose |
|--------|---------|
| `scripts/check_camera.sh` | List V4L devices and modes |
| `scripts/camera_paths.py` | Print stable by-path camera symlinks |
| `scripts/export_model.py` | Export `.pt` → ncnn / openvino / hailo |
| `scripts/compile_hailo_hef.sh` | Hailo DFC compile helper |
| `scripts/probe_esp_uart.py` | PING/listen on serial |
| `scripts/simulate_esp_serial.py` | Fake ESP for UI dev |
| `scripts/test_telemetry.py` | Inject fake telemetry |

### Tests

```bash
python3 -m unittest discover -s tests -p 'test_*.py' -v
# Or run individual modules:
python3 -c "from tests.test_sub_motion import *; ..."
python3 -c "from tests.test_xbox_mapping import *; ..."
```

Key test files: `test_sub_motion.py`, `test_xbox_mapping.py`, `test_model_runtime.py`, `test_stereo.py`, `test_hailo_runtime.py`, `test_app.py`.

---

## 18. Troubleshooting

### Dashboard / YOLO

| Symptom | Fix |
|---------|-----|
| Model dropdown does nothing | Hard refresh (`Ctrl+Shift+R`); ensure Auto mode; check `/sub/api/models` |
| Actuators stay at zero in Auto | Confirm mode badge shows Auto; check YOLO status line; verify detections |
| YOLO start timeout | Restart `~/sub`; check Hailo/NCNN export exists |
| Faders don't track YOLO | Must be Auto mode with inference running; check SSE connected |

### Cameras

| Symptom | Fix |
|---------|-----|
| Black / wrong camera | Use by-path devices; run `check_camera.sh` |
| Stereo range always null | Both cams must detect target; check `match_max_dy_px` |
| Range consistently wrong | Tune `range_scale` or `fov_h_deg` |

### ESP / serial

| Symptom | Fix |
|---------|-----|
| ESP offline badge | Check USB cable/port; `probe_esp_uart.py`; verify firmware flashed |
| `S2` not moving servos | Check PCA9685 I2C (`TEL pca9685` in serial log) |
| Write errors / reconnect loop | Only one process on serial port; check GPS vs ESP port conflict |

### Xbox

| Symptom | Fix |
|---------|-----|
| Pad not detected | `python -m src.xbox_controller`; check Bluetooth pair |
| Drift triggers override | Raise `deadzone` / `override_actuator_threshold` |
| Linked flap no effect | Check badge **Flap — LINKED**; verify aft steer moving |

### Hailo

| Symptom | Fix |
|---------|-----|
| Hailo unavailable in catalog | Check HailoRT install; verify `.hef` in `weights/<id>/hailo/` |
| Second model slow | Expected — only one HEF on device; second uses NCNN |

---

## Related docs

| Doc | Contents |
|-----|----------|
| [`README.md`](../README.md) | Quick start |
| [`STATUS.md`](../STATUS.md) | Feature checklist |
| [`docs/ESP32_SERIAL.md`](ESP32_SERIAL.md) | Full serial protocol |
| [`docs/PI_ESP32_COMPATIBILITY.md`](PI_ESP32_COMPATIBILITY.md) | Pre-flight checklist |
| [`docs/sub_endpoints_plan.md`](sub_endpoints_plan.md) | API design notes |
| [`esp32/README.md`](../esp32/README.md) | Firmware build & pins |
| [`weights/README.md`](../weights/README.md) | Model folder conventions |
