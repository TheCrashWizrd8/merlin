# Thruster and Blue Robotics Basic ESC

End-to-end description of how this vehicle drives the main thruster. Authoritative firmware is `esp32/sub_rc/sub_rc.ino`. Pin summary is in [WIRING.md](WIRING.md).

## Hardware

| Item | Detail |
|------|--------|
| ESC | [Blue Robotics Basic ESC](https://bluerobotics.com/store/thrusters/speed-controllers/besc30-r3/) (bidirectional, center stop) |
| Signal path | Pi → ESP32-S3 (USB serial) → I2C → **PCA9685 channel 0** → ESC white (signal) |
| Ground | ESC signal **black** must share ground with PCA/ESP/Pi |
| Legacy L298N | GPIO 13, 12, 6 are **not** used; firmware holds them LOW |

Do not use fin-style PCA ticks (150–600) on channel 0. Those pulses are ~730–2930 µs and the Basic ESC will not initialize correctly.

## Blue Robotics PWM (what the ESC expects)

From the [thruster usage guide](https://bluerobotics.com/learn/thruster-usage-guide/):

| Pulse width | Meaning |
|-------------|---------|
| **1500 µs** | Stop / initialize (must be sent continuously for a few seconds after power-on) |
| **1525–1900 µs** | Forward |
| **1475–1100 µs** | Reverse |
| Deadband | **1500 ± 25 µs** |

PWM must **stay on** (up to ~400 Hz). If the signal drops, the ESC re-runs its startup sequence.

### Beep sequence (motor connected, ESC powered)

1. **Three** rising beeps — motor phases detected  
2. **One** beep — valid throttle band seen (1100–1900 µs)  
3. **One** beep — **1500 µs stop** recognized → armed  

If you only hear one beep after the three power beeps, the ESC is seeing a pulse in range but **not** 1500 µs stop (wrong width or intermittent throttle).

## PCA9685 timing (firmware)

Servos on channels 1–5 use tick calibration from `config/pins.yaml` (`tick_us: 4.8828`). The thruster uses **microsecond targets**, converted to PCA ticks:

- Oscillator: nominally 25 MHz, but it varies per board. Measured by ear on this board: tick 307 (25 MHz) and tick 320 (26 MHz) missed the ESC's final 1500 µs stop beep, tick 314 (**25.5 MHz**, `ESC_OSC_MHZ_DEFAULT`) gave it, so the firmware uses 25.5 MHz. Retune live with `ESC OSC <MHz>` on the serial console (it drops the signal 1.5 s so the ESC replays its beeps). Symptom of a wrong value: power beeps but no final 1500 µs beep, and the motor creeps at idle.
- Frame rate: **50 Hz** → prescale register **121** (`PCA9685_PWM_PRESCALE_50HZ`)
- Tick duration: **(prescale + 1) / osc** (≈ 4.78 µs at 25.5 MHz)
- Tick from µs: `tick = us × osc_MHz / (prescale + 1)`

At prescale **121** and 25.5 MHz, stop is tick **314** ≈ **1500 µs**.  
(Using prescale **135** with tick 307 was ~**1670 µs** — enough for beep 2 in the table above but not for the 1500 µs init beep.)

Firmware constants (`sub_rc.ino`):

| Constant | Value | Role |
|----------|-------|------|
| `ESC_US_STOP` | 1500 | Arm + idle |
| `ESC_US_REVERSE` | 1100 | Full reverse (LT) |
| `ESC_US_FORWARD` | 1900 | Full forward (RT) |
| `ESC_ARM_HOLD_MS` | 5000 | Pi throttle blocked until this much continuous stop |
| `ESC_PWM_REFRESH_MS` | 20 | Re-write ch0 ~50 Hz (`escMaintainOutput`) |

On PCA init the ESP prints e.g. `OK ESC BlueRobotics stop tick=307 ~1500us prescale=121`.

## Firmware behavior

### Command input

Thruster is the `X` field on the actuator line:

```text
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>\n
```

`thruster` is **−1..1**. Mapping to pulse width is linear: 0 → 1500 µs, +1 → 1900 µs, −1 → 1100 µs (`escTickForCommand` / `escUsToTick`).

`TEST T <val>` runs the same path for bench tests.

### Arming gate (software)

The Basic ESC arms on **1500 µs** from the ESC’s point of view. The firmware also enforces a **5 s** window where:

1. Commanded throttle must stay at **0** (within `MOTOR_DEADBAND`, 0.02).  
2. Channel 0 streams **1500 µs** (`escUpdateNeutralHold` + `escMaintainOutput`).  
3. If RT/LT (or Pi) commands non-zero throttle before the gate opens, output stays at stop and the timer **resets**.  
4. After 5 s: `escThrottleAllowed = true`, serial line `OK ESC ready (Basic ESC init beeps should be done)`.

While the gate is closed, `setThruster` still drives **1500 µs** to the ESC; only the **commanded** `hw.thruster` value is forced to 0 until ready.

After the gate is open, non-zero throttle is allowed; releasing triggers returns to 1500 µs stop without disarming the ESC (unless the ESC itself times out—keep PWM streaming).

### Legacy GPIO

`PIN_THR_IN1` / `IN2` / `PWM` (13, 12, 6) are not used for the thruster.

## Raspberry Pi stack

| Layer | Role |
|-------|------|
| `config/xbox_mapping.yaml` | RT forward, LT reverse; **RT wins** if both pressed |
| `src/xbox_mapping.py` | `_map_thruster()`; trigger deadzone ~0.08 |
| `src/xbox_controller.py` | LT/RT read as SDL axes (not mis-scaled as −1..1 sticks) |
| `src/esp_bridge.py` | Builds `S2 … X <thruster>`; keepalive ~1 s; parses `TEL esc` |
| `src/sub_state.py` | `telemetry.thruster.esc`: `ready`, `neutral_ms`, `arm_hold_ms`, `pulse_us`, `message` |
| Dashboard | ESC badge, thrust panel status, serial **highlights** (`OK ESC`, `S2`, `TEL esc`) |

Logical actuator name: **`thrusterX`** in [-1, 1]. Emergency stop (Xbox **B** / dashboard) zeros movement including thrust.

### Telemetry

```text
TEL esc <ready 0|1> <neutral_ms> <arm_hold_ms> <pulse_us>
```

`pulse_us` is the measured width of the last ESC pulse written on ch0 (sanity check: **~1498–1502** at idle).

Diagnostic lines:

- `OK ESC arming (1500us stop, release RT/LT)`  
- `OK ESC ready (Basic ESC init beeps should be done)`  

## Operator checklist

1. Flash `esp32/sub_rc/sub_rc.ino` and run the Pi stack (`~/sub` or your usual entrypoint).  
2. Power the ESC (three phase beeps).  
3. **Release RT and LT**; thrust slider at 0; wait **~5 s**.  
4. Confirm dashboard **ESC — READY** and `pulse_us` ≈ **1500**; listen for **two** init beeps.  
5. Use **RT** / **LT** for forward / reverse.

If the badge shows **blocked**, the HUD shows Pi thrust and trigger values—fix those before the arm timer can finish.

## Troubleshooting

| Symptom | Likely cause |
|---------|----------------|
| Three beeps only | No valid PWM yet, or ESC signal/ground not connected to PCA ch0 |
| One beep after three, no second | Pulse not 1500 µs (check `pulse_us` on dashboard; reflash if prescale wrong) |
| `OK ESC ready` but motor idle | ESC not armed (listen for 5 beeps); or throttle still 0 on wire |
| Arms then full throttle | Pi sending ~1.0 on `X`; check triggers, manual slider, mode |
| Reverse weak / same as stop | Command below reverse threshold; ESC deadband around 1500 µs |

## Related files

- `esp32/sub_rc/sub_rc.ino` — ESC ticks, arming, `escMaintainOutput`  
- `config/pins.yaml` — `pca9685.channels.thruster_esc`, `l298n.thruster` note  
- `config/xbox_mapping.yaml` — `thruster:` triggers  
- `docs/WIRING.md` — physical pins and PCA channel map  
- `esp32/README.md` — short operator notes  
