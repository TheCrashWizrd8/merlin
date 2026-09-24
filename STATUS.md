# Project Status

_Last updated: 24 September 2026_

---

## What This Project Does

An autonomous apple-detection RC submarine on a Raspberry Pi 5. Dual USB endoscope cameras feed a YOLO model (Hailo or NCNN) that tracks apples; steering, thruster, and ballast commands guide the sub toward a target. A unified **sub vehicle stack** on the same Pi handles ESP32 telemetry, layered actuator control, Xbox gamepad input, GPS, and a pilot web dashboard at `/sub/`.

---

## Current Status

### Done

| Area | Detail |
|------|--------|
| **Unified launcher** | `~/sub` → `run.py` → `src/app.py` — one process for dashboard, cameras, ESP, GPS, Xbox, optional YOLO |
| **Inference service** | `src/inference_service.py` — start/stop/switch YOLO from dashboard without restart |
| **Camera** | Dual FIT0819 USB endoscopes (stereo). Centres **16 cm** apart, **67°** FOV. Working size 640×480 |
| **Inference pipeline** | Camera(s) → YOLO → tracker → optional stereo range → controller → hardware / web stream |
| **Hailo backend** | `backend: hailo` in `config/model.yaml`; HEF export via `scripts/compile_hailo_hef.sh` |
| **NCNN backend** | `backend: ncnn`; export via `scripts/export_model.py`. Pi 5: ~68 ms vs ~304 ms PyTorch |
| **Stereo range** | Shared YOLO model; left/right frames; `src/stereo.py` triangulates → HUD range (metres) |
| **Trained model** | **`detect`:** `weights/best.pt` + NCNN/Hailo exports. Catalog supports multiple models |
| **YOLO model picker** | `/sub/` **Auto (YOLO)** → model buttons; API `GET/POST /sub/api/models` |
| **Controller** | Proportional + sustain-until-centred steering/tilt; size-based drive mapping |
| **Telemetry safety** | Leak stop, low-battery drive scale, alignment gating |
| **Unified sub serial** | `interface: sub` → `SubBridgeOutput` → `sub_state`; single `esp_bridge` owns UART |
| **Layered sub motion** | `src/sub_motion.py` — gyro fins → aft steer → thruster + ballast height trim |
| **Linked flap** | YOLO and Xbox share linked flap mode (fins oppose aft-steer axis); default on in `hardware.yaml` |
| **Xbox controller** | RT/LT thrust, RB/LB ballast, D-pad linked flap; **B = emergency stop** (halt YOLO + zero actuators) |
| **Pilot HUD** | `/sub/` — SSE live feed, MJPEG camera, fader sync with YOLO actuators, diagnostics |
| **Web stream** | MJPEG at `/video_feed`; snapshot at `/snapshot` |
| **ESP32 sub firmware** | `esp32/sub_rc/` — PCA9685 servos, L298N thruster, ballast, leak/ADC/IMU telemetry |
| **ESP bridge** | `src/esp_bridge.py` — `TEL` lines in, `S2`/`B` at ~20 Hz |
| **Tests** | `tests/test_sub_motion.py`, `tests/test_xbox_mapping.py`, `tests/test_app.py`, stereo, detector, etc. |

### In Progress

| Area | Detail |
|------|--------|
| **Field testing** | End-to-end YOLO auto + layered sub actuators on real hardware |
| **I2C peripherals** | PCA9685 @ 0x40 + MPU6050 @ 0x68 on shared SDA/SCL |

### Not Started / Future

| Area | Detail |
|------|--------|
| **Spektrum AR8020T receiver** | RC receiver decoding on ESP32 |
| **Stereo calibration** | Tune `fov_h_deg` / `focal_length_px` against known distance |
| **Runtime pin config** | `config/pins.yaml` is reference-only |
| **Motion phase on dashboard** | `level` / `point` / `approach` not yet shown as UI badge |

---

## How to Run

### Everything (recommended)

```bash
~/sub
# Dashboard: http://<pi-ip>:8080/sub/
# YOLO stream (when started): http://<pi-ip>:8080/
# YOLO off by default — pick Auto (YOLO) + model on dashboard, or:
~/sub --yolo --backend hailo --timing
```

Install once: `ln -sf ~/yolo-project/sub ~/sub`

### Common flags

```bash
~/sub --help
~/sub --serial-port /dev/ttyACM0
~/sub --no-xbox --no-gps
~/sub --no-camera          # dashboard only
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
| `config/model.yaml` | `backend` | `hailo` or `ncnn` |
| `config/model.yaml` | `active_model` | `detect` |
| `config/hardware.yaml` | `interface` | `sub` |
| `config/hardware.yaml` | `sub_motion.linked_flap` | `true` |
| `config/hardware.yaml` | `sub_serial.port` | `/dev/ttyACM0` or `/dev/serial0` |
| `config/xbox_mapping.yaml` | `emergency_stop.button` | `b` |

Full reference: **`docs/GUIDE.md`** and **`README.md`**.

---

## Architecture (high level)

```
~/sub → run.py → src/app.py
                    │
    ┌───────────────┼───────────────┐
    │               │               │
    ▼               ▼               ▼
 sub_web      inference_service   xbox_controller
 (SSE HUD)    (YOLO start/stop)   (manual override, B stop)
    │               │               │
    └───────────────┼───────────────┘
                    ▼
              sub_state + sub_motion
         (linked flap, layered actuators)
                    │
                    ▼
              esp_bridge → sub_rc.ino
```

**Control modes:** `manual`, `xbox`, `auto`. Xbox override during YOLO uses filtered actuators + threshold 0.12. **B** stops inference and zeros all movement.

---

## Project Structure

```
yolo-project/
├── sub                     # Shell entry point (install as ~/sub)
├── run.py                  # Python launcher (same process)
├── sub_server.py           # Alias → run.py
├── inference.py            # Alias → run.py
├── src/app.py              # Unified main()
├── src/inference_service.py
├── src/sub_motion.py
├── src/sub_dashboard.html
├── config/                 # model, hardware, pins, xbox_mapping
├── esp32/sub_rc/
├── weights/
└── docs/GUIDE.md
```
