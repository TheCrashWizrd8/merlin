---

## title: "RC Receiver + Sensor Endpoints Plan (`--car` / `--sub`)"
updated: 2026-03-25
status: draft

# RC Receiver + Sensor Endpoints Plan

This document captures what is currently implemented in `yolo-project`, what was discovered while exploring it, and a non-invasive plan for adding new endpoints/control outputs for an RC-controlled vehicle.

The key requirement is to add capabilities without impacting the current “car” behavior. Current YOLO-based control continues to run as **auto**, and manual control continues to run as **manual**.

---

## 1. What exists today (current architecture)

### 1.0 Current directory state (what you have right now)

At a high level, this repo is already set up to run YOLO inference on the Pi and send only a single actuator triple to hardware:

- `inference.py`: main camera → YOLO → controller loop, and (optionally) the Flask web server
- `src/controller.py`: computes `ControlOutput` with exactly `steering_servo`, `drive_motor`, `camera_tilt_servo` (all normalized `-1.0..+1.0`)
- `src/control_source.py`: holds the current “mode” (`auto` vs `manual`) and the current manual `S/D/T` values
- `src/web_stream.py`: Flask endpoints:
  - `GET/POST /api/control` to set `mode` + manual `s/d/t`
  - `GET /video_feed` for the MJPEG stream
- `src/hardware.py`: hardware abstraction:
  - `interface: serial` sends one line per frame to an ESP32 in the format `S <steer> D <drive> T <tilt>\n`
- `esp32/apple_car_rc/apple_car_rc.ino`: ESP32 firmware that parses only `S/D/T` serial lines and drives:
  - 2 servos (steer/tilt) via PCA9685
  - 1 DC motor via L298N (sustained speed using `D`)

Also note:

- There is **no Spektrum/AR8020T receiver decoding code** yet.
- There is **no ultrasonic/leak/gyro code** yet.
- There is **no structured telemetry protocol** yet from ESP32 → Pi; the only serial protocol defined is the Pi → ESP32 actuator command line.

### 1.1 Pi runtime control loop

The main loop lives in `[inference.py](/home/subs/yolo-project/inference.py)`. Each frame does:

1. Camera frame capture (`src/camera.py`)
2. YOLO detection (`src/detector.py`)
3. Target selection/tracking (`src/tracker.py`)
4. Compute actuator commands (`src/controller.py`)
5. Send actuator commands to configured hardware (`src/hardware.py`)
6. Optionally serve video/API when started with `--web`

### 1.2 `ControlOutput` is the single actuator-command container

Actuator commands are represented by `[src/controller.py#ControlOutput](/home/subs/yolo-project/src/controller.py)`. It currently contains exactly three normalized actuator fields:

- `steering_servo` in range `-1.0 .. +1.0`
- `drive_motor` in range `-1.0 .. +1.0`
- `camera_tilt_servo` in range `-1.0 .. +1.0`

Plus detection/telemetry-ish fields for display (e.g., `error_x`, `error_y`, bbox details).

### 1.3 Manual vs auto is controlled by `src/control_source.py`

Manual/auto selection uses:

- `[src/control_source.py#set_mode](/home/subs/yolo-project/src/control_source.py)`
- `[src/control_source.py#get_current_sdt](/home/subs/yolo-project/src/control_source.py)`

Important behaviors:

- `auto` mode returns the S/D/T values derived from YOLO (`controller.compute(...)`).
- `manual` mode returns the S/D/T values set by the web endpoint today (or an optional RC callback if registered).

At the moment, the RC callback mechanism exists but is not wired to any physical Spektrum receiver parsing logic. Also, `src/control_source.py` is currently hard-coded to `SDT` (three values). Any future `--sub` extra axes (aft steer Y/Z, thruster X, fore fins) will need a separate data model/mapping than the existing `SDT` triple.

### 1.4 Flask endpoints (existing “car” control UI)

When launched with `--web`, the Flask server is implemented in `[src/web_stream.py](/home/subs/yolo-project/src/web_stream.py)`.

Existing endpoints:

- `GET /api/control`
  - Returns JSON: `{ mode, s, d, t }`
- `POST /api/control`
  - Accepts JSON that can update:
    - `mode` (`auto` or `manual`)
    - `s`, `d`, `t` (clamped floats)
  - Returns JSON with the effective values after update
- `GET /video_feed`
  - MJPEG stream of the annotated camera feed

The existing UI (embedded in `web_stream.py`) only manipulates `s/d/t` (steer/drive/tilt).

### 1.5 Hardware output mapping (car actuators)

The hardware abstraction lives in `[src/hardware.py](/home/subs/yolo-project/src/hardware.py)`.

It supports:

- `interface: stub` (no hardware; logs only)
- `interface: pca9685` (3 PWM channels on PCA9685)
- `interface: serial` (Pi -> microcontroller serial protocol)

For the current setup, the serial interface is the one that actually drives the ESP32 firmware.

#### Current serial protocol (Pi -> ESP32)

The Pi always sends one line per frame when `interface: serial`:

`S <steer> D <drive> T <tilt>\n`

This happens in `[inference.py](/home/subs/yolo-project/inference.py)` where it formats the string and calls:

- `hardware.apply(output)` (steering/drive/tilt)
- plus it optionally prints the exact `serial_line` when serial verbose is enabled

The ESP32 firmware is in `[esp32/apple_car_rc/apple_car_rc.ino](/home/subs/yolo-project/esp32/apple_car_rc/apple_car_rc.ino)` and parses:

- `sscanf(line, "S %f D %f T %f", ...)`

It then drives:

- Steering servo (PCA9685 channel 0)
- Camera tilt servo (PCA9685 channel 1)
- DC motor via L298N (direction pins + PWM speed)

Safety timeout:

- If Pi stops sending for `SERIAL_TIMEOUT_MS` (8000 ms in the current sketch comments), the motor stops.
- Servos keep their last commanded positions.

---

## 2. What is NOT present yet

During exploration, there is currently no code for:

- Spektrum AR8020T (DSM/DM) receiver decoding on the Pi or ESP32
- Any structured telemetry packet/format beyond console printouts and the MJPEG video stream
- Ultrasonic sensor reading
- Leak sensor reading
- Gyro/IMU reading

So, “new endpoints” must be introduced as stubs/non-invasive placeholders unless and until receiver/sensor parsing is implemented.

---

## 3. Confirmed `--car` vs `--sub` behavior (design decisions so far)

### 3.1 Keep existing manual/automatic mode model

Current design already has two modes:

- YOLO auto: controller output drives `S/D/T`
- Web/controller manual: manual `S/D/T` values drive `S/D/T`

Confirmed request:

- Keep this model.
- In `--sub` mode, YOLO remains auto and “receiver-driven manual” becomes the manual input source.

Clarification for implementation: the concept of “`--car` mode” does not exist today as a CLI flag. Practically, you can treat the current behavior as “car default” (YOLO auto + web manual on `/api/control`); `--sub` is what adds extra endpoints/UI and a new actuator mapping.

### 3.2 Receiver connection location

Confirmed request:

- The Spektrum AR8020T receiver will be connected/decoded by the ESP32 (recommended).
- The Pi will be responsible only for:
  - selecting `--sub` endpoint behavior
  - exposing endpoints (including stubs for sensors/telemetry until wired)
  - optionally forwarding control values depending on the final design

### 3.3 `--sub` actuator set (axes described)

Confirmed request for the sub vehicle actuators:

- **Aft steering axes**:
  - `aft steer y` (labeled as `Y`)
  - `aft steer z` (labeled as `Z`)
- **Thruster**:
  - `thruster x` (labeled as `X`)
- **Fore cap fins**:
  - `fin left`
  - `fin right`

The current channel layout is not finalized; for now, endpoint/controller scaffolding should not depend on the receiver channel numbers.

### 3.4 Sub auto behavior (YOLO steering mirroring)

Confirmed request:

- In `auto`, YOLO should drive/mirror aft steering axes.
- Auto mapping selected: `mirror-car`
  - Thruster can be driven from YOLO drive (`controller.drive_motor`).
  - Steering servo mirroring to aft axes should follow the “car steering meaning” until you confirm direction/inversion.
- In auto, fins should remain neutral (or be explicitly defined as neutral/mirrored once you confirm your desired stabilization behavior).

---

## 4. Non-invasive endpoint plan (stubs + safe extension)

The implementation goal is “more ways to control” for the receiver and additional actuator outputs, selected with `--sub` vs `--car`.

## 4.1 CLI selection for non-invasive behavior

Proposal (to match your confirmed preference):

- Add a `--sub` CLI flag to `[inference.py](/home/subs/yolo-project/inference.py)`.
- Default behavior remains the existing “car” pipeline when `--sub` is not provided.

Non-invasive requirement:

- Existing endpoints and actuator mappings for the car must remain unchanged.
- Any new endpoints should be:
  - namespaced (recommended), or
  - added only when `--sub` is enabled

Suggested namespace (recommended):

- Car endpoints remain as-is:
  - `/api/control` and `/video_feed`
- Sub endpoints live under:
  - `/sub/api/control` (receiver-controlled manual UI/data)
  - `/sub/api/receiver` (receiver status / channel values stubs)
  - `/sub/api/sensors` (ultrasonic / leak / gyro readings stubs)
  - `/sub/api/actuators` (sub actuator command echo stubs)

This avoids breaking clients that currently call `/api/control`.

## 4.2 Endpoint categories to add

### A) Receiver endpoint(s)

Purpose:

- Provide “what the receiver is currently commanding”.
- When real receiver parsing exists on the ESP32, these endpoints return decoded values.

Suggested response shapes:

- `GET /sub/api/receiver`
  - Return a JSON object like:
    - `mode` (receiver connected/active/idle, until defined)
    - `channels`: `{ ch1: ..., ch2: ... }` or a list
    - `timestamp`

Until receiver parsing is implemented, return placeholders:

- `connected: false`
- `channels: {}` (or nulls)
- `timestamp`

### B) Sensor endpoint(s)

Purpose:

- Expose ultrasonic, leak, and gyro/IMU readings.
- Initially stubs (no impact to current code).

Suggested endpoints:

- `GET /sub/api/sensors/ultrasonic` -> `{ distance_mm: null, connected: false }`
- `GET /sub/api/sensors/leaks` -> `{ leaks: [null,...], connected: false }`
- `GET /sub/api/sensors/gyro` -> `{ pitch: null, roll: null, yaw: null, connected: false }`

Until sensor code exists on ESP32:

- return `connected: false` and `null` values

### C) Actuator endpoint(s) / mapping echo

Purpose:

- Make it clear which actuators are being driven and what values they get from:
  - receiver (manual)
  - YOLO (auto)

Suggested:

- `GET /sub/api/actuators`
  - Return:
    - `auto`: `{ aftSteerY: ..., aftSteerZ: ..., thrusterX: ..., finLeft: ..., finRight: ... }`
    - `manual`: same keys
    - `effective`: the currently active set (auto or manual)

If you want this to remain non-invasive:

- Start by reporting “effective” values derived from whatever internal mapping exists.
- If manual values are not yet available from receiver decoding, report `null` or mirror-only values.

## 4.2.1 API contract snapshot (stub responses)

This is a “stable contract” you can use while implementing. Even before receiver telemetry exists, endpoints should return JSON with the same shape so the UI can be developed without waiting on hardware.

### `GET /sub/api/receiver` (placeholders initially)

```json
{
  "connected": false,
  "mode": "idle",
  "channels": {},
  "timestamp": 0.0
}
```

### `GET /sub/api/sensors/ultrasonic` (placeholders initially)

```json
{
  "connected": false,
  "distance_mm": null,
  "timestamp": 0.0
}
```

### `GET /sub/api/sensors/leaks` (placeholders initially)

Keep a stable shape even if you later add more leak channels.

```json
{
  "connected": false,
  "leaks": [],
  "timestamp": 0.0
}
```

### `GET /sub/api/sensors/gyro` (placeholders initially)

```json
{
  "connected": false,
  "pitch": null,
  "roll": null,
  "yaw": null,
  "timestamp": 0.0
}
```

### `GET /sub/api/actuators` (effective values)

```json
{
  "mode": "auto",
  "effective": {
    "aftSteerY": 0.0,
    "aftSteerZ": 0.0,
    "thrusterX": 0.0,
    "finLeft": 0.0,
    "finRight": 0.0
  },
  "auto": {
    "aftSteerY": 0.0,
    "aftSteerZ": 0.0,
    "thrusterX": 0.0,
    "finLeft": 0.0,
    "finRight": 0.0
  },
  "manual": {
    "aftSteerY": null,
    "aftSteerZ": null,
    "thrusterX": null,
    "finLeft": null,
    "finRight": null
  },
  "timestamp": 0.0
}
```

## 4.3 How to keep existing code untouched

Non-invasive strategy:

- Keep car control path exactly as-is:
  - current Flask routes (`/api/control`)
  - current `ControlOutput` and `S/D/T` actuator mapping
  - current hardware serial protocol `S/D/T` to ESP32

For `--sub`:

Option 1 (recommended for minimal impact):

- Introduce new API routes that do not affect existing `/api/control`.
- Introduce new internal mapping paths that use separate functions/classes, without changing the car pipeline.

Option 2 (more invasive):

- Extend `ControlOutput` to carry more actuator axes.
- This is higher-risk for regressions.

Given your requirement “no impact on current code”, prefer option 1 initially (API stubs + mapping scaffolding).

Requirements:

- Must NOT modify existing "S %f D %f T %f" parsing

- Must be parsed in a separate conditional block on ESP32

- Must be sent only when --sub is enabled

---

## 5. Implementation scaffolding (future work, clearly marked TODO)

This section is a proposal for the next coding steps once receiver and sensor wiring details are confirmed.

### TODO 5.1 Receiver decoding integration (ESP32 -> Pi)

You said to connect the AR8020T receiver to the ESP32, but receiver signal type/pins are not finalized yet.

Next steps to implement:

- On ESP32, implement receiver parsing for AR8020T:
  - DSM/DM decoding or telemetry extraction, depending on the wiring.
- Decide a transport of decoded receiver channels/telemetry to the Pi:
  - simplest: extend the existing serial protocol (new message types) OR
  - add a second serial line format (still over USB serial).

Because today’s ESP32 firmware only parses lines of the format `S ... D ... T ...`, any receiver->Pi communication must be additive.

Non-invasive requirement:

- Do not change the existing `S/D/T` line parsing semantics.

### TODO 5.2 Add `--sub` flag and sub route handling

Steps:

1. Add `--sub` to `[inference.py](/home/subs/yolo-project/inference.py)`.
2. When `--web` + `--sub`:
  - register additional Flask routes under `/sub/...`.
3. Ensure the existing `--web` car UI still works with no changes.
4. Because you requested a new sub web UI page, also implement a new route that serves a sub-specific HTML/JS page (for example `GET /sub/` or `GET /sub/control`) instead of reusing the existing embedded car UI in `[src/web_stream.py](/home/subs/yolo-project/src/web_stream.py)`.

### TODO 5.3 Manual source for sub (`controller manual`) from receiver

You confirmed:

- In manual mode, receiver channels drive sub actuators.

Current Python manual source uses `src/control_source.py`:

- `get_current_sdt(...)` returns either auto SDL or manual S/D/T values.
- There is an RC callback mechanism (`set_rc_source`) but it is currently unused.

Future:

- Wire `--sub` manual to a receiver callback.
- This should remain independent from car `s/d/t`.

Practical note: since `src/control_source.py` is SDT-only, you will likely add a parallel “sub control source” that stores the extra axes (aft steer Y/Z, thruster X, fins L/R) and exposes them to a new mapping layer.

### TODO 5.4 Auto mirroring to aft steering axes (from YOLO)

You confirmed:

- auto should mirror steering from car control logic into aft steering axes.

Future tasks:

- Define sign/inversion for:
  - `aftSteerY` from `steering_servo`
  - `aftSteerZ` from `camera_tilt_servo` (or other mapping, depending on how you want Z to behave)
- Define fin behavior:
  - neutral by default (initially)
  - or optionally mirror for roll stabilization after you test.

Practical note: right now YOLO only produces 2D error (`error_x`, `error_y`). Mapping these onto aft steering Y and Z (and deciding what Z means physically) is mostly a sign/inversion + axis convention task. Treat it as a configuration-driven mapping once you verify directions on real hardware.

Requirements:

- Must NOT modify existing "S %f D %f T %f" parsing

- Must be parsed in a separate conditional block on ESP32

- Must be sent only when --sub is enabled



### TODO 5.5 Concrete implementation sequence (non-invasive)

This is a suggested implementation order that keeps the current “car” behavior unchanged and pushes any new sub functionality behind `--sub`.

1. Add `--sub` CLI flag (Pi)
  - File: `[inference.py](/home/subs/yolo-project/inference.py)`
  - Add `parser.add_argument("--sub", action="store_true", help="Enable sub vehicle endpoints/control")`
  - Keep the default path exactly as today when `--sub` is not set.
  - In the existing `if args.web:` block, pass `args.sub` into the Flask app registration so it can decide which routes/UI to add.
2. Keep the existing car UI working (`/`)
  - File: `[src/web_stream.py](/home/subs/yolo-project/src/web_stream.py)`
  - Do not change the existing `/` HTML/JS page and do not change the existing car endpoints:
    - `GET/POST /api/control`
    - `GET /video_feed`
  - Instead, add new routes for the sub UI:
    - `GET /sub/` (or `/sub/control`) to serve a new HTML/JS page
    - `GET/POST /sub/api/control` (receiver/manual + mode toggle for sub)
    - Optional: `GET /sub/api/status` to expose `mode` + connection state for UI
3. Introduce a “sub control state” separate from SDT
  - File: `[src/control_source.py](/home/subs/yolo-project/src/control_source.py)`
  - Keep existing SDT (`s/d/t`) code path untouched for the car.
  - Add a parallel store for sub axes (names are up to you; the important part is that it does not reuse SDT):
    - `aftSteerY`
    - `aftSteerZ`
    - `thrusterX`
    - `finLeft`
    - `finRight`
  - Add functions analogous to `set_mode`, `get_mode`, `set_manual`, `get_current_sdt`, but for sub:
    - `set_sub_mode('auto'|'manual')`
    - `set_sub_manual(...)`
    - `get_sub_effective_axes(auto_axes, manual_axes)`
4. Wire auto (YOLO) -> sub actuators without affecting car
  - File: `[src/controller.py](/home/subs/yolo-project/src/controller.py)`
  - Do not change the current `ControlOutput` structure yet if you want minimal risk.
  - Instead, add a small “mapping function” that converts existing car outputs into sub axes for auto:
    - `aftSteerY` derived from `steering_servo`
    - `aftSteerZ` derived from `camera_tilt_servo` (with inversion/sign to be tuned)
    - `thrusterX` derived from `drive_motor` (range/mapping for your thruster may differ from DC motor; for now treat it as equivalent `-1..+1` -> `-1..+1`)
    - `finLeft`/`finRight` default neutral in auto (unless you decide to mirror roll during initial tuning)
5. Wire manual (receiver) -> sub actuators (stubs first)
  - File(s): start in `[src/control_source.py](/home/subs/yolo-project/src/control_source.py)` only
  - Because receiver parsing is not implemented yet, implement manual mode semantics as “placeholders”:
    - Sub manual inputs return fixed neutral defaults (or values you set via the sub UI)
  - When ESP32 receiver decoding is later implemented, you will replace the stub manual source with receiver-derived values.
6. Define how new sub axes reach hardware (ESP32 vs Pi PWM/serial)
  - Current state: `src/hardware.py` + ESP32 firmware only understand `S/D/T`.
  - Non-invasive approach:
    - For now, keep `hardware.apply(output)` unchanged for car.
    - For `--sub`, implement a separate serial message format (additive), for example:
      - `S2 <aftY> <aftZ> F <finL> <finR> X <thruster> \n`
    - Update ESP32 firmware to parse the new message in addition to the existing `S/D/T`.
  Requirements:
  - Must NOT modify existing "S %f D %f T %f" parsing
  - Must be parsed in a separate conditional block on ESP32
  - Must be sent only when --sub is enabled
  - This step is where you must be most careful, but it is additive: the existing `S/D/T` parsing remains unchanged.
7. Add ultrasonic/leak/gyro API endpoints as stubs only
  - File: `[src/web_stream.py](/home/subs/yolo-project/src/web_stream.py)`
  - Return `connected: false` and `null`/empty arrays for now.
  - Later, when ESP32 sends telemetry, add a serial reader on the Pi and replace stubs with real values.

### Acceptance checklist (Cursor-friendly)

Use this checklist to verify the changes are correct while implementing. Each item should be checked after its corresponding step.

1. Car behavior unchanged (default)
  - Run: `python inference.py --headless`
  - Verify: existing endpoints still work:
    - `GET/POST http://<pi-ip>:8080/api/control` updates `mode` + `s/d/t`
    - `GET http://<pi-ip>:8080/video_feed` streams MJPEG
  - Verify: when `--sub` is not enabled, Pi->ESP32 actuator commands are still only:
    - `S <steer> D <drive> T <tilt>\n`
2. `--sub` gates additional UI/routes only
  - Run: `python inference.py --web --headless --sub`
  - Verify:
    - car UI at `/` still loads
    - sub UI route(s) exist (e.g. `/sub/` or `/sub/control`) and are served successfully
    - sub endpoints under `/sub/api/...` return valid JSON without touching the existing `/api/control` handler
3. Two-mode model still holds
  - Verify:
    - YOLO is still `auto`
    - web/controller is still `manual`
  - Until receiver parsing exists, verify sub manual defaults are neutral placeholders and do not crash the app.
4. Sub actuator mapping scaffolding is isolated
  - Verify:
    - car SDT mapping code path is unchanged
    - sub mapping logic is implemented separately (so car doesn’t regress)
5. Sub hardware transport is additive (when implemented)
  - Verify (after ESP32 firmware update):
    - the original `sscanf("S %f D %f T %f", ...)` path still works
    - the new sub serial message format is accepted without breaking the existing one
6. Sensor endpoints are safe stubs initially
  - Verify each returns stable, non-blocking JSON:
    - `GET /sub/api/sensors/ultrasonic` -> `connected: false`, `distance_mm: null`
    - `GET /sub/api/sensors/leaks` -> `connected: false` and a stable shape for leaks
    - `GET /sub/api/sensors/gyro` -> `connected: false` and `pitch/roll/yaw: null`

---

## 6. Open questions / unknowns (so implementation can proceed)

1. Receiver wiring/pinouts:
  - What signal type/pin the AR8020T provides to ESP32 (DSM pulse into GPIO, or UART telemetry, etc.) is still undecided.
2. Receiver channel mapping:
  - You provided actuator intent but confirmed channel numbers are not important yet.
  - Implementation should allow flexible mapping in config later.
3. Thruster control type on the sub:
  - You described `thruster x` but current plan is mostly about endpoint scaffolding and stubs for telemetry.
  - It may require a different output mapping than car’s DC motor `D` unless the ESP32 firmware is extended.
4. Fin roll stabilization:
  - Whether fins should be neutral in auto or actively mirrored requires a test plan.
5. Endpoint response formats:
  - The JSON shapes above are suggestions for stubs; final schema should match whatever you plan to visualize/control next.

---

## 7. References to current code paths (for quick navigation)

- Flask server + web routes: `[src/web_stream.py](/home/subs/yolo-project/src/web_stream.py)`
- Manual/auto state: `[src/control_source.py](/home/subs/yolo-project/src/control_source.py)`
- Actuator computation: `[src/controller.py](/home/subs/yolo-project/src/controller.py)`
- Hardware abstraction: `[src/hardware.py](/home/subs/yolo-project/src/hardware.py)`
- Pi main loop: `[inference.py](/home/subs/yolo-project/inference.py)`
- ESP32 serial parsing & actuator driving: `[esp32/apple_car_rc/apple_car_rc.ino](/home/subs/yolo-project/esp32/apple_car_rc/apple_car_rc.ino)`

