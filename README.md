# YOLO Apple RC Sub

_Last updated: 24 September 2026_

Autonomous apple-tracking submarine stack for **Raspberry Pi 5**. USB endoscope cameras feed YOLO (Hailo-8L, NCNN, or PyTorch); a layered controller drives aft steer, fore fins, thruster, and ballast through an **ESP32-S3** hub. A web dashboard at `/sub/` provides the pilot HUD, diagnostics, and live telemetry.

**Full documentation:** [`docs/GUIDE.md`](docs/GUIDE.md)

---

## What it does

| Layer | Role |
|-------|------|
| **Vision** | Detect & track apples (or gates/other classes) on stereo + FOV cameras |
| **Control** | `ControlOutput` → layered sub motion (fins, steer, thrust, ballast) |
| **Hardware** | Pi ↔ ESP32 serial (`S2` actuators, `B` ballast, `TEL` telemetry) |
| **Operator** | Web dashboard, Xbox controller, or manual sliders |

Stereo pair (16 cm baseline, 67° FOV) gives **range in metres** when both cameras see the target.

---

## Quick start

```bash
cd ~/yolo-project
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
ln -sf ~/yolo-project/sub ~/sub   # once — start from anywhere

# Edit config/hardware.yaml — serial port, camera paths, interface: sub
~/sub
```

Open **http://\<pi-ip\>:8080/sub/**

| URL | Content |
|-----|---------|
| `/sub/` | Pilot HUD + diagnostics (scroll down) |
| `/` | Live MJPEG (FOV camera) |
| `/snapshot/fov` | FOV JPEG for HUD background |
| `/snapshot/yolo_left`, `/snapshot/yolo_right` | Stereo YOLO feeds (lazy-loaded in diagnostics) |

YOLO is **off** until you choose **Auto (YOLO)** and pick a model, or start with:

```bash
~/sub --yolo --backend hailo --timing
```

`~/sub` wraps `run.py` in the project venv. `python run.py`, `sub_server.py`, and `inference.py` are equivalent if run from `~/yolo-project`.

---

## Control modes

| Mode | Source | Notes |
|------|--------|-------|
| **Manual** | Dashboard sliders | Bench default; values persist in browser |
| **Xbox** | Gamepad (`config/xbox_mapping.yaml`) | Auto-connect on pad plug-in |
| **Auto (YOLO)** | Inference + `sub_motion.py` | Model dropdown in HUD; hot-swap models live |

**Xbox while YOLO runs:** deliberate pad input temporarily overrides actuators (configurable). **B button** = emergency stop (YOLO off, all movement zeroed).

**Linked flap** (default ON): fore fins oppose aft-steer angle — same for Xbox and YOLO auto. Toggle with **D-pad left**.

---

## Configuration

| File | Purpose |
|------|---------|
| [`config/model.yaml`](config/model.yaml) | Model catalog, backend, confidence, `active_model` |
| [`config/hardware.yaml`](config/hardware.yaml) | Cameras, serial, Xbox, control gains, `sub_motion` |
| [`config/xbox_mapping.yaml`](config/xbox_mapping.yaml) | Stick/button → actuators, linked flap, emergency stop |
| [`config/pins.yaml`](config/pins.yaml) | Pin reference (ESP32, PCA9685, L298N) |
| [`config/dataset.yaml`](config/dataset.yaml) | Training data path |
| [`weights/<id>/`](weights/) | One folder per model — see [`weights/README.md`](weights/README.md) |

---

## Models

Add a model by copying `weights/_template` → `weights/<id>/`, drop in `best.pt`, edit `model.yaml`, export:

```bash
python scripts/export_model.py --weights weights/<id>/best.pt --format hailo
python scripts/export_model.py --weights weights/<id>/best.pt --format ncnn
```

Switch from the dashboard (**Auto** → **Model** dropdown) or API:

```bash
curl -X POST http://localhost:8080/sub/api/models/select \
  -H 'Content-Type: application/json' \
  -d '{"id":"detect","backend":"hailo"}'
```

Hailo-8L runs **one HEF** at a time; a second Hailo model falls back to NCNN on CPU.

---

## Training

Train on GPU / [Google Colab](https://colab.research.google.com/) — not on the Pi. See [`docs/GUIDE.md` §5–6](docs/GUIDE.md) and [`train_colab.ipynb`](train_colab.ipynb).

```bash
python train.py   # after config/dataset.yaml is set
```

---

## ESP32 firmware

Flash from the Pi:

```bash
bash esp32/upload_from_pi.sh scan
bash esp32/upload_from_pi.sh
```

Firmware: [`esp32/sub_rc/`](esp32/sub_rc/). Protocol: [`docs/ESP32_SERIAL.md`](docs/ESP32_SERIAL.md).

---

## Project layout

```
sub                          # Shell entry point → run.py (install as ~/sub)
run.py / src/app.py          # Python launcher (same process)
src/inference_service.py     # YOLO start/stop/switch (dashboard-driven)
src/model_runtime.py         # Model catalog + hot-swap
src/sub_motion.py            # Layered auto actuators + linked flap
src/sub_dashboard.html       # Pilot HUD + diagnostics
src/esp_bridge.py            # Pi ↔ ESP32 serial
src/xbox_controller.py       # Gamepad thread
config/                      # YAML configuration
weights/<id>/                  # Per-model weights + exports
esp32/sub_rc/                # Vehicle firmware
docs/GUIDE.md                # Full system guide
```

---

## Useful commands

```bash
~/sub -h                                      # All CLI flags
python -m src.xbox_controller                 # Test gamepad
bash scripts/check_camera.sh                  # Camera nodes
python scripts/probe_esp_uart.py              # ESP serial probe
python scripts/test_telemetry.py              # Fake telemetry (no hardware)
bash esp32/upload_from_pi.sh                  # Flash ESP32
```

---

## Related docs

| Doc | Contents |
|-----|----------|
| [`docs/GUIDE.md`](docs/GUIDE.md) | Installation, architecture, dashboard, Xbox, API, troubleshooting |
| [`docs/ESP32_SERIAL.md`](docs/ESP32_SERIAL.md) | Serial protocol |
| [`esp32/README.md`](esp32/README.md) | Firmware pins & upload |
| [`weights/README.md`](weights/README.md) | Model folder layout |
| [`STATUS.md`](STATUS.md) | Feature checklist (may lag behind code) |
