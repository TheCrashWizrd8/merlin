# Pi ↔ ESP32 serial protocol

The Raspberry Pi communicates with an **ESP32-S3** on the RC submarine over serial using the **`sub_rc`** firmware and **`esp_bridge.py`**.

---

## Architecture overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Raspberry Pi 5                                │
│                                                                       │
│  inference.py ──► hardware.py (SubBridgeOutput) ──► sub_state        │
│                                                                       │
│  sub_server.py / inference --sub ──► esp_bridge.py                   │
│                                       /dev/serial0 or /dev/ttyACM0   │
│                                       S2 / B / CAL / PING + TEL rx    │
└───────────────────────────────────────┬───────────────────────────────┘
                                        │ GPIO UART or USB
                                        ▼
                                   sub_rc.ino
                                   (full sub vehicle)
```

| Link | Pi device | Baud | Firmware | Purpose |
|------|-----------|------|----------|---------|
| **Sub vehicle** | `/dev/serial0` (GPIO UART) or `/dev/ttyACM0` (USB) | 115200 | `esp32/sub_rc/` | Actuators, ballast, sensors, telemetry |

Config in `config/hardware.yaml`:

```yaml
interface: sub
sub_serial:
  port: "/dev/serial0"   # or /dev/ttyACM0 for USB
  baud_rate: 115200
```

Pin reference: **`config/pins.yaml`**.

---

## Sub vehicle protocol

Used by `sub_server.py` and `inference.py --sub` → `src/esp_bridge.py`.

### Wiring (GPIO UART)

Cross-connect Pi header pins to ESP32:

| Pi | Header pin | ESP32 (sub_rc) |
|----|------------|----------------|
| GPIO14 TX | 8 | RX (GPIO 44) |
| GPIO15 RX | 10 | TX (GPIO 43) |
| GND | 6 | GND |

Enable UART on the Pi (`/dev/serial0`). On Pi 5 this is typically available when serial console is disabled in `config.txt`.

Probe with:

```bash
python scripts/probe_esp_uart.py
```

### Pi → ESP commands

Sent at ~20 Hz by the ESP bridge writer thread (unless diagnostic mode is active):

**Ballast** (fore and aft tanks, -1.0 … +1.0):

```
B <fore> <aft>\n
```

Example: `B 1.000 -1.000` — fill fore, drain aft.

On the ESP, each tank uses **INA + INB** (digital on/off, no PWM) plus a **3-wire linear pot** (3.3 V, wiper → ADC, GND). See `config/pins.yaml` and `esp32/README.md`.

**Sub actuators** (all values -1.0 … +1.0):

```
S2 <aftY> <aftZ> F <finL> <finR> X <thruster>\n
```

Example: `S2 0.120 -0.050 F 0.000 0.000 X 0.450`

| Token | Actuator |
|-------|----------|
| `S2` + two floats | Aft steer Y, aft steer Z (PCA9685 ch 0/1) |
| `F` + two floats | Fore fin left, fore fin right (PCA9685 ch 2/3) |
| `X` + one float | Thruster via L298N |

**Calibration & diagnostics** (on demand via dashboard or `POST /sub/api/serial`):

```
PING
PINS
CAL B <fore|aft> top|bottom|show
TEST S <ch 0-3> <val -1..1>
TEST T <val -1..1>
TEST B <fore|aft> fill|drain|stop
TEST L
TEST A
HELP
```

### ESP → Pi telemetry

The ESP32 sends `TEL` lines at ~5 Hz. The Pi parser in `esp_bridge.py` updates `sub_state`:

```
TEL battery 12.45
TEL gyro 1.2 -0.5 45.0
TEL depth 3.45
TEL leak 0 0 1 0
TEL ballast fore 0.500 2048 1 FILL
TEL ballast aft 0.400 1638 0 STOP
TEL ballastcal fore 500 3500 1
TEL controls
<7 value lines>
TEL thruster 0.400 128
TEL status READY
TEL fault NONE
TEL heartbeat 42
```

Diagnostic replies (not prefixed with `TEL`):

```
OK PONG
OK PINS BEGIN
... pin lines ...
OK PINS END
OK TEST servo ch=0 val=0.000
OK CAL B fore top 3500
ERR TEST ...
```

Telemetry is marked stale after 3 s with no updates.

### Bench testing (no sensors wired)

With **`sub_rc`** powered and UART linked but **no battery divider, depth sensor, pots, or motors** connected:

| Line | Typical bench value | Interpretation |
|------|---------------------|----------------|
| `TEL status READY` | present | Firmware running |
| `TEL fault NONE` | present | No leak alarm |
| `TEL heartbeat N` | N increments ~5 Hz | Live telemetry loop |
| `TEL leak 0 0 0 0` | all zero | OK (`PIN_LEAK = -1` or pulled low) |
| `TEL gyro 0 0 0` | zeros | No IMU — hardcoded in firmware |
| `TEL thruster 0 0` | zeros | Idle |
| `B 0.000 0.000` | zeros | Pi sending ballast stop (manual mode) |
| `S2 0 … 0` | all zeros | Pi sending idle actuators (manual mode) |
| `TEL battery 12–14` | high | Floating ADC × voltage divider scale — **ignore** |
| `TEL depth ~3.3` | high | Floating ADC voltage — **ignore** |
| `TEL ballast … 4095` or low ADC | varies | Floating pot pins — **ignore** until calibrated |
| `TEL ballastcal … -1 -1 0` | uncalibrated | Run dashboard **Cal top/bottom** when pots are wired |

**Quick link test:** `PING` → `OK PONG`.

**Non-zero `S2` on the bench:** those lines are **Pi → ESP** commands. If you see e.g. `S2 0.350 …`, the Pi is sending stick/slider input — switch dashboard to **Manual** and center sliders, or disconnect the gamepad.

### Firmware

**`esp32/sub_rc/`** — primary submarine sketch. See **`esp32/README.md`** for pinout and upload.

---

## Safety

| Firmware | Timeout | Behaviour |
|----------|---------|-----------|
| `sub_rc` | 8000 ms | Thruster stops; ballast holds last command |

Reduce max motor speed in firmware (`MOTOR_MAX_SPEED` / `MOTOR_PWM_MAX`).

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| No USB serial device | `ls /dev/ttyACM* /dev/ttyUSB*` — ESP32 must be plugged in via USB |
| No GPIO UART | `ls -l /dev/serial0` — enable Pi UART; run `probe_esp_uart.py` |
| Sub dashboard shows disconnected | Correct `sub_serial.port`? ESP flashed with `sub_rc` and `USE_PI_UART=1`? |
| Wrong port in config | Override: `python sub_server.py --serial-port /dev/serial0` |
| Garbled telemetry | Baud must be 115200 on both sides |

---

## Related docs

- **`esp32/README.md`** — Build, upload, pinout
- **`docs/PI_ESP32_COMPATIBILITY.md`** — Pre-flight checklist
- **`config/pins.yaml`** — Authoritative pin map
- **`docs/GUIDE.md`** §14–16 — Sub system usage
