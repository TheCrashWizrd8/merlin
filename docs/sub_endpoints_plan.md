# Sub Dashboard API Reference

> **Status:** Implemented (August 2026)  
> The sub vehicle endpoints described here are live in `src/sub_web.py`.

---

## Overview

The sub stack provides a namespaced web UI and REST API under `/sub/`.

| Entry point | Command |
|-------------|---------|
| Sub only (no YOLO) | `python sub_server.py` |
| YOLO + sub auto-mirror | `python inference.py --web --sub` |
| Simulated telemetry | `python scripts/test_telemetry.py` |

Dashboard: **`http://<pi-ip>:8080/sub/`**

---

## Control modes

| Mode | Source | Set via |
|------|--------|---------|
| `manual` | Web UI / API (**default**) | Dashboard sliders, `POST /sub/api/control` |
| `xbox` | Xbox/gamepad | **Xbox** tab when a controller is connected |
| `auto` | YOLO mirror | Active when `inference.py --sub` is running |

**Safety defaults**

- Server starts in **`manual`** mode with zero actuators and zero ballast (`src/sub_state.py`).
- In **`xbox`** mode with no pad connected, the Pi sends **manual slider values**, not stale stick input.
- On pad disconnect, xbox actuator values are zeroed and manual setpoints are used again.
- LT/RT ballast bias only applies in **`xbox`** mode with a live controller.

**Dashboard persistence**

The web UI (`src/sub_dashboard.html`) stores control state in browser **`sessionStorage`** (`subDashboardControl`):

- Control mode (Manual / Xbox / Auto)
- Manual actuator slider values
- Fore/aft ballast slider values

On page load, saved values are restored to the UI and POSTed to the API before the live stream connects. Stream updates refresh telemetry bars only — they do not reset slider positions. Use a hard refresh to reset to manual/all-zero.

---

## Live data (SSE)

The dashboard uses **Server-Sent Events** instead of REST polling.

| Item | Detail |
|------|--------|
| Endpoint | `GET /sub/api/stream` |
| Format | `text/event-stream` — `event: update` with JSON payload |
| Trigger | Server pushes when `sub_state` changes (ESP telemetry, Xbox, serial log, control writes) |
| Rate cap | ~30 Hz max; 15 s heartbeat comment keeps the connection alive |
| Initial snapshot | Full dashboard payload sent immediately on connect |

**Browser:** `EventSource('/sub/api/stream')` in `src/sub_dashboard.html`. On disconnect, the browser auto-reconnects.

**Camera:** The dashboard camera panel uses native MJPEG at **`/video_feed`** (~15 FPS) — not snapshot polling.

Individual REST routes below remain available for scripts, curl, and debugging.

#### `GET /sub/api/stream` — `event: update` payload

```json
{
  "telemetry": { "...": "same shape as GET /sub/api/telemetry" },
  "control": { "...": "same shape as GET /sub/api/control" },
  "status": { "...": "same shape as GET /sub/api/status" },
  "serial": { "lines": [ { "ts": 1699999999.0, "dir": "rx", "line": "TEL battery 12.45" } ] },
  "pins": {
    "expected": { "...": "from config/pins.yaml" },
    "esp_pins_lines": [],
    "esp_pins_map": {},
    "last_pong_ts": null
  },
  "diagnostics": { "...": "same shape as GET /sub/api/diagnostics" },
  "version": 42
}
```

`version` increments on every state change; clients can use it to detect missed events (the server coalesces bursts within the rate cap).

---

## YOLO auto mapping

YOLO auto mapping (`src/sub_control.py`):

```
steering_servo      → aftSteerY
camera_tilt_servo   → aftSteerZ
drive_motor         → thrusterX
(fins stay neutral)
```

---

## API routes

### Dashboard

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/` | GET | Sub dashboard HTML (`src/sub_dashboard.html`) |
| `/sub/api/stream` | GET | **SSE live feed** — telemetry, control, status, serial, pins, diagnostics |
| `/video_feed` | GET | MJPEG camera stream (~15 FPS when camera is active) |
| `/snapshot` | GET | Single JPEG frame (debug/scripts; dashboard uses `/video_feed`) |

### Status & telemetry

REST routes below mirror fields in the SSE `update` payload.

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/status` | GET | ESP/Xbox connection, leak alarm, control mode |
| `/sub/api/telemetry` | GET | All telemetry (battery, gyro, depth, leaks, ballast) |
| `/sub/api/telemetry/battery` | GET | Battery voltage |
| `/sub/api/telemetry/gyro` | GET | Pitch, roll, yaw |
| `/sub/api/telemetry/depth` | GET | Depth reading |
| `/sub/api/telemetry/leaks` | GET | Leak sensor array + triggered flag |
| `/sub/api/telemetry/ballast` | GET | Fore/aft tank levels and commands |

#### `GET /sub/api/status` response

```json
{
  "esp_connected": true,
  "xbox_connected": false,
  "leak_alarm": false,
  "control_mode": "xbox",
  "telemetry_age_s": 0.4,
  "timestamp": 1710000000.0
}
```

#### `GET /sub/api/telemetry/leaks` response

```json
{
  "timestamp": 1710000000.0,
  "connected": true,
  "sensors": [false, false, true, false],
  "triggered": true
}
```

### Control (read)

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/control` | GET | Full control snapshot |
| `/sub/api/actuators` | GET | Effective/auto/manual/xbox-mapped actuators |
| `/sub/api/xbox` | GET | Raw Xbox stick/button state |

#### `GET /sub/api/actuators` response

```json
{
  "mode": "auto",
  "effective": {
    "aftSteerY": 0.12,
    "aftSteerZ": -0.05,
    "thrusterX": 0.45,
    "finLeft": 0.0,
    "finRight": 0.0
  },
  "auto": { "...": "..." },
  "manual": { "...": "..." },
  "xbox_mapped": { "...": "..." },
  "timestamp": 1710000000.0
}
```

### Control (write)

| Route | Method | Body | Description |
|-------|--------|------|-------------|
| `/sub/api/control` | POST | `{ "mode": "manual", "aftSteerY": 0.5, ... }` | Set mode and/or manual actuators |
| `/sub/api/control/actuators` | POST | `{ "aftSteerY": 0.5, "thrusterX": 0.3, ... }` | Set manual actuators (forces manual mode) |
| `/sub/api/control/ballast` | POST | `{ "action": "fill", "tank": "fore" }` | Ballast fill/drain/stop |
| `/sub/api/control/ballast` | POST | `{ "fore": 1.0, "aft": -1.0 }` | Raw fore/aft commands (-1..+1) |
| `/sub/api/ballast/calibrate` | POST | `{ "tank": "fore", "end": "top" }` | Send `CAL B fore top` to ESP |
| `/sub/api/ballast/calibrate/resume` | POST | — | Resume normal bridge operation |

Ballast actions: `fill` (+1), `drain` (-1), `stop`/`neutral`/`hold` (0). Tank: `fore`, `aft`, or `both`.

### Serial monitor & diagnostics

The dashboard **ESP Serial Monitor** panel shows the recent Pi ↔ ESP log from `sub_state` serial buffer.

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/serial` | GET | Recent serial log (`?limit=200`, default 100) |
| `/sub/api/serial` | POST | `{ "line": "PING" }` — send raw command |

**Log entry shape:** `{ "ts": 1699999999.0, "dir": "rx"|"tx"|"sys", "line": "..." }`

**Dashboard UI behaviour**

- Tall scroll area (~52vh, min 320px) with styled scrollbar.
- Auto-scroll only when scrolled to the bottom; scrolling up or selecting text pauses the live feed (SSE keeps running; UI skips DOM updates until you scroll back).
- Green = ESP → Pi (`TEL …`, `OK PONG`), blue = Pi → ESP (`S2 …`, `B …`), yellow = bridge system messages.

**Bench check (nothing wired):** expect `TEL status READY`, incrementing `TEL heartbeat`, `TEL leak 0 0 0 0`, and idle `B 0 0` / `S2 … 0` from the Pi. Battery, depth, and ballast ADC values are often nonsense until sensors and calibration are wired. Send `PING` → `OK PONG`.

| Route | Method | Description |
|-------|--------|-------------|
| `/sub/api/pins` | GET | Expected pins (from `config/pins.yaml`) vs ESP report |
| `/sub/api/diagnostics` | GET | Last PONG, PINS capture, last diagnostic reply |
| `/sub/api/test` | POST | `{ "command": "PING" }` — single diagnostic command |
| `/sub/api/test/resume` | POST | Exit diagnostic mode |
| `/sub/api/test/run` | POST | Run full pin confirmation checklist |

Pin checklist commands (automated by `/sub/api/test/run`):

```
PING → OK PONG
PINS → OK PINS
TEST S 0 0 … TEST S 3 0
TEST T 0
TEST B fore stop / TEST B aft stop
TEST L / TEST A
```

---

## Serial protocols (Pi ↔ ESP32)

See **`docs/ESP32_SERIAL.md`** for full detail.

**Pi → ESP (sent by `esp_bridge.py` at ~20 Hz):**

```
B <fore> <aft>
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>
```

**ESP → Pi (telemetry at ~5 Hz):**

```
TEL battery 12.45
TEL gyro 1.2 -0.5 45.0
TEL depth 3.45
TEL leak 0 0 1 0
TEL ballast fore 0.500 2048 1 FILL
TEL thruster 0.400 128
TEL status READY
TEL fault NONE
TEL heartbeat 42
```

---

## Architecture

```
Browser (/sub/)
    │
    ├── EventSource ──► GET /sub/api/stream  (SSE push on state change)
    ├── <img src="/video_feed">              (MJPEG camera)
    └── fetch POST ──► /sub/api/control …    (user actions)
                         │
                         ▼
                    sub_web.py (Flask routes)
                         │
                         ▼
                    sub_state.py (thread-safe state + change notifications)
                         ▲
                         │
              esp_bridge.py ◄──► /dev/serial0 ◄──► sub_rc.ino
                         │
              xbox_controller.py (optional)
                         │
              inference.py (interface: sub) ──► sub_motion.plan_sub_motion()
                                              ──► auto mode + ballast
                                              ──► esp_bridge (single serial)
                                              ──► set_latest_frame() → /video_feed
```


---

## Source files

| File | Role |
|------|------|
| `sub_server.py` | Standalone sub dashboard entry point |
| `src/sub_web.py` | Flask route registration |
| `src/sub_state.py` | Shared telemetry/control state |
| `src/sub_control.py` | YOLO → sub actuator mapping |
| `src/esp_bridge.py` | Serial reader/writer threads |
| `src/xbox_controller.py` | Gamepad input |
| `src/sub_dashboard.html` | Web UI |
| `esp32/sub_rc/sub_rc.ino` | ESP32 firmware |

---

## Not yet implemented

| Feature | Notes |
|---------|-------|
| Spektrum AR8020T receiver decoding | Manual mode uses web/Xbox for now |
| I2C depth sensor | ADC placeholder on GPIO 3 |
| IMU telemetry | Parser ready; hardware integration TBD |
| Runtime consumption of `config/pins.yaml` | Reference only; firmware defines pins |

---

## Related docs

- **`docs/GUIDE.md`** §14–16 — Usage guide
- **`docs/ESP32_SERIAL.md`** — Protocol reference
- **`config/pins.yaml`** — Pin map
