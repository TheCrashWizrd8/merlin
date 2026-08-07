# ESP32-S3 firmware — Apple RC sub

Primary sketch: **`sub_rc/`** — submarine receiver, actuators, ballast, leak sensors, and telemetry over **Pi GPIO UART** (or USB serial).

Hardware test: **`sub_hw_test/`** — isolated actuator bring-up.

---

## Sketches

| Sketch | Pi connection | Protocol | When |
|--------|---------------|----------|------|
| **`sub_rc/`** | GPIO UART (`/dev/serial0`) or USB (`/dev/ttyACM0`) | `S2`, `B`, `TEL` | Production sub build |
| **`sub_hw_test/`** | Either | Manual | Wiring verification |

---

## Requirements

- **Board:** ESP32-S3 (e.g. ESP32-S3-DevKitC, Pro S3)
- **Arduino IDE** or **PlatformIO**, with **ESP32** board support (Espressif)
- **Libraries:**
  - Adafruit PWM Servo Driver — for PCA9685 (when `ENABLE_PCA9685=1` in `sub_rc`)

---

## Pinout — `sub_rc/` (default)

Edit `#define` values at the top of **`sub_rc/sub_rc.ino`** to match your wiring. Authoritative reference: **`config/pins.yaml`**.

### Pi UART (default: `USE_PI_UART=1`)

| ESP32 GPIO | Function | Connects to |
|------------|----------|-------------|
| 44 | RX | Pi GPIO14 TX (header pin 8) |
| 43 | TX | Pi GPIO15 RX (header pin 10) |

### I2C / PCA9685 (set `ENABLE_PCA9685=1` when wired)

| ESP32 GPIO | Function |
|------------|----------|
| 21 | I2C SDA |
| 22 | I2C SCL |

**Do not use GPIO 26–32 or 29/30** on ESP32-S3 — those are internal flash/PSRAM.

| PCA9685 channel | Actuator |
|-----------------|----------|
| 0 | Aft steer Y |
| 1 | Aft steer Z |
| 2 | Fore fin left |
| 3 | Fore fin right |

### Thruster (L298N)

| ESP32 GPIO | L298N pin |
|------------|-----------|
| 4 | IN1 |
| 12 | IN2 |
| 6 | Enable PWM |

### Ballast tanks

Each tank: **5 wires** — INA, INB, pot wiper (ADC), pot 3.3 V, pot GND.  
Fill/drain is **on/off** via INA/INB (no PWM).

| Tank | INA GPIO | INB GPIO | Pot ADC |
|------|----------|----------|---------|
| Fore | 13 | 14 | 7 |
| Aft | 9 | 8 | 11 |

| State | INA | INB |
|-------|-----|-----|
| Fill | HIGH | LOW |
| Drain | LOW | HIGH |
| Stop | LOW | LOW |

### Sensors

| ESP32 GPIO | Sensor |
|------------|--------|
| 1 | Battery ADC |
| 3 | Depth ADC (or I2C depth later) |
| 5 | Leak detector (active HIGH = leak) |

---

## Wiring (sub_rc)

```
Pi header pin 8  (GPIO14 TX) ──► ESP GPIO 44 (RX)
Pi header pin 10 (GPIO15 RX) ◄── ESP GPIO 43 (TX)
Pi GND ──────────────────────── ESP GND

PCA9685 SDA/SCL ──► ESP GPIO 21/22
Servo signals ────► PCA9685 channels 0–3
L298N IN1/IN2/PWM ► ESP GPIO 4/12/6
Ballast INA/INB ──► ESP GPIO 13/14 (fore), 9/8 (aft) + pot wiper on 7/11
Leak sensor ──────► ESP GPIO 5
Battery / depth ──► ESP ADC GPIO 1/3
```

USB can remain connected for serial debug (`USB_DEBUG_MIRROR=1` mirrors key messages to USB Serial).

---

## Build and upload

### Option A: From the Raspberry Pi (Arduino CLI)

**1. One-time setup:**

```bash
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
```

**2. Build and upload sub firmware** (from project root):

```bash
arduino-cli compile --clean --fqbn esp32:esp32:esp32s3 esp32/sub_rc
arduino-cli board list
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 esp32/sub_rc
```

Or use the helper script:

```bash
bash esp32/upload_from_pi.sh          # auto-detect USB port
bash esp32/upload_from_pi.sh scan     # list ports only
```

> **Note:** Upload uses the ESP32's **USB port** (`/dev/ttyACM0`). After flashing, runtime communication for `sub_rc` uses **GPIO UART** to the Pi (or USB if configured).

### Option B: From a PC with Arduino IDE

1. Board: **ESP32S3 Dev Module**
2. Open `sub_rc/sub_rc.ino`
3. Install **Adafruit PWM Servo Driver** (when PCA9685 is wired)
4. Upload via USB

After upload, open Serial Monitor at **115200 baud** to see startup messages.

---

## Pi configuration

In `config/hardware.yaml`:

```yaml
interface: sub
sub_serial:
  port: "/dev/serial0"   # or /dev/ttyACM0 for USB
  baud_rate: 115200
```

Run sub dashboard:

```bash
python sub_server.py
# or
python inference.py --web --sub
```

Open **http://\<pi-ip\>:8080/sub/**

Default control mode is **manual** (sliders at zero). The dashboard serial monitor shows live `TEL` (ESP → Pi) and `S2`/`B` (Pi → ESP) lines. With nothing wired, expect `TEL status READY`, rising `TEL heartbeat`, and idle `B 0 0` / `S2 … 0` — battery, depth, and ballast ADC values are often floating-pin noise until sensors are connected and ballast is calibrated. See **`docs/ESP32_SERIAL.md`** § Bench testing.

---

## Troubleshooting

| Error | Fix |
|-------|-----|
| `Failed to connect to ESP32-S3: No serial data received` | Wrong upload port — use `arduino-cli board list`; hold **BOOT** during upload if needed |
| `partitions.bin: No such file or directory` | Clean build: `arduino-cli compile --clean ...` |
| Sub dashboard disconnected | Check GPIO UART wiring; run `python scripts/probe_esp_uart.py` |
| ESP watchdog resets | Don't use GPIO 26–32 or 29/30 for I2C — use 21/22 |
| PCA9685 not responding | Set `ENABLE_PCA9685=1` in sketch after wiring SDA/SCL to 21/22 |

---

## Safety

- **Serial timeout:** 8000 ms — thruster/motor stops if Pi stops sending.
- **Motor cap:** Lower `MOTOR_MAX_SPEED` (default 255) for bench testing.
- **Leak sensor:** GPIO 5 active HIGH triggers leak alarm in telemetry and dashboard.

---

## Related docs

- **`docs/ESP32_SERIAL.md`** — Full protocol reference
- **`config/pins.yaml`** — Pin map synced from firmware
- **`docs/GUIDE.md`** §14–16 — Sub system on the Pi
