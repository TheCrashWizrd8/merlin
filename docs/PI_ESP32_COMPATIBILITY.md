# Pi ↔ ESP32 compatibility checklist

Use this to confirm the Raspberry Pi and ESP32 firmware match before your first run.

---

## Sub vehicle (GPIO UART or USB)

Used by `sub_server.py` and `inference.py` (when `interface: sub`) → `src/esp_bridge.py`.  
**Single serial owner:** `SubBridgeOutput` → `sub_state` → `esp_bridge` (no duplicate S/D/T client).

### Wiring

| Pi | ESP32 (`sub_rc`, `USE_PI_UART=1`) |
|----|-----------------------------------|
| GPIO14 TX (pin 8) | RX (GPIO 44) |
| GPIO15 RX (pin 10) | TX (GPIO 43) |
| GND | GND |

### Protocol — Pi → ESP

| Item | Pi (`esp_bridge.py`) | ESP32 (`sub_rc.ino`) | Match |
|------|----------------------|----------------------|-------|
| **Actuators** | `S2 <y> <z> F <fl> <fr> X <thr>\n` | `sscanf(..., "S2 %f %f F %f %f X %f", ...)` | ✅ |
| **Ballast** | `B <fore> <aft>\n` | Parsed separately | ✅ |
| **Diagnostics** | `PING`, `PINS`, `TEST …`, `CAL B …` | Handled in command parser | ✅ |
| **Baud rate** | 115200 | 115200 | ✅ |
| **Port** | `/dev/serial0` or `/dev/ttyACM0` | PiLink UART or USB CDC | ✅ |
| **Send rate** | ~20 Hz (actuators + ballast) | Parsed each line | ✅ |

### Protocol — ESP → Pi

| Item | ESP32 | Pi (`esp_bridge.py`) | Match |
|------|-------|----------------------|-------|
| **Telemetry prefix** | `TEL …` | `parse_telemetry_line()` | ✅ |
| **Battery** | `TEL battery 12.45` | Parsed | ✅ |
| **Gyro** | `TEL gyro p r y` | Parsed | ✅ |
| **Depth** | `TEL depth 3.45` | Parsed | ✅ |
| **Leaks** | `TEL leak 0 0 1 0` | Parsed | ✅ |
| **Ballast** | `TEL ballast fore …` | Parsed | ✅ |
| **Heartbeat** | `TEL heartbeat N` | Parsed | ✅ |
| **PONG** | `OK PONG` | `parse_diagnostic_line()` | ✅ |

### Pi config

```yaml
interface: sub
sub_serial:
  port: "/dev/serial0"
  baud_rate: 115200
```

### Quick test

```bash
# 1. Probe UART (listen + PING)
python scripts/probe_esp_uart.py

# 2. Start sub dashboard
python sub_server.py

# 3. Open http://<pi-ip>:8080/sub/
#    — ESP connected flag should go green
#    — Telemetry panels should update (~5 Hz)
#    — POST PING via serial monitor → OK PONG
```

Or run the built-in pin checklist from the dashboard (**Run all tests**) or:

```bash
curl -X POST http://localhost:8080/sub/api/test/run
```

---

## Value semantics

| Signal | -1.0 | 0.0 | +1.0 |
|--------|------|-----|------|
| **Steer / aftSteerY** | Full left | Straight | Full right |
| **Drive / thrusterX** | Full reverse | Stop | Full forward |
| **Tilt / aftSteerZ** | Full down | Level | Full up |
| **Ballast** | Drain | Hold/stop | Fill |

### Control modes (sub dashboard)

| Mode | Default? | Pi sends |
|------|----------|----------|
| `manual` | **Yes** | Dashboard slider values (`S2 …`, `B …`) |
| `xbox` | No | Live gamepad mapping when connected; falls back to manual sliders if pad offline |
| `auto` | During inference | `sub_motion.plan_sub_motion()` → `S2 …`, `B …` (fins, steer, thruster, ballast) |

Inference sets **`auto`** on startup. See **`docs/GUIDE.md`** §15 for layered motion detail.

Dashboard slider/mode state persists in browser **`sessionStorage`** across normal reloads. Server-side default is **`manual`** (`src/sub_state.py`).

---

## Sub hardware (sub_rc firmware)

| Component | Connection |
|-----------|------------|
| Aft steer Y/Z, fins | PCA9685 @ 0x40 on I2C GPIO 29/30 |
| IMU (pitch/roll) | MPU6050 GY-521 @ 0x68 on same I2C bus |
| Thruster | L298N — GPIO 13 (IN1), 12 (IN2), 6 (PWM) |
| Fore ballast | GPIO 16 DIR A, 15 PWM A, 10 pot ADC |
| Aft ballast | GPIO 4 DIR B, 5 PWM B, 3 pot ADC |
| Leak | GPIO 1 (IO1 / D1) — 4-zone leak board, combined active HIGH |
| Battery | ADC GPIO 2 |
| Depth | ADC GPIO 7 |

Full map: **`config/pins.yaml`**.

---

## Common mismatches

| Problem | Check |
|---------|-------|
| Sub dashboard disconnected | GPIO UART wired? Flash `sub_rc` with `USE_PI_UART=1`? Port `/dev/serial0`? |
| Upload works but sub UART silent | Upload uses USB; runtime uses GPIO UART — check pins 8/10 wiring |
| Wrong steer direction | Set `INVERT_STEER` / inversion flags in `.ino` and reflash |
| Motor runs when it shouldn't | Serial timeout (8 s) stops thruster; verify Pi is sending |
| No telemetry | Run `probe_esp_uart.py`; check ESP is powered and flashed with `sub_rc` |
| Rogue actuator commands on bench | Default mode is `manual`; center dashboard sliders; disconnect gamepad or set mode Manual |
| Non-zero `S2` in serial monitor | Pi → ESP TX lines — not ESP sensor data; check control mode and sliders |
| PCA9685 servos dead | Confirm PCA9685 + MPU6050 share SDA/SCL on GPIO 29/30; boot should print `OK PCA9685` |
| Gyro always zero | Confirm GY-521 on same I2C bus @ 0x68; boot should print `OK MPU6050` |

---

## Summary

- **First sub test:** `probe_esp_uart.py` → `sub_server.py` → open `/sub/` → run pin checklist.
- **First YOLO test:** `inference.py --web --timing` with trained `weights/best.pt` (when `interface: sub`).
- **YOLO + sub:** layered auto motion — fins, aft steer, thruster, ballast height (`docs/GUIDE.md` §15).
