# ESP32-S3 firmware — Apple RC car receiver

Receives `S <steer> D <drive> T <tilt>` from the Raspberry Pi over USB serial and drives:

- **Steering** — Hitec HS-646WP servo  
- **Camera tilt** — Hitec HS-646WP servo  
- **Drive** — DC motor via L298N (IN1, IN2, Enable PWM)

---

## Requirements

- **Board:** ESP32-S3 (e.g. ESP32-S3-DevKitC, Pro S3)
- **Arduino IDE** or **PlatformIO**, with **ESP32** board support (Espressif)
- **Library:** [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) — install via Arduino Library Manager: search **ESP32Servo**

---

## Pinout (default — change in `.ino` if needed)


| GPIO | Function       | Connects to              |
| ---- | -------------- | ------------------------ |
| 4    | Steering servo | Hitec HS-646WP signal    |
| 5    | Tilt servo     | Hitec HS-646WP signal    |
| 6    | L298N IN1      | Direction (forward high) |
| 7    | L298N IN2      | Direction (reverse high) |
| 8    | L298N Enable A | PWM speed (0–255)        |


- Servos: 5 V and GND from a suitable supply (not from the L298N 5 V if it’s logic-only).
- L298N: motor supply and motor outputs to the DC motor; logic (IN1, IN2, Enable) from ESP32 3.3 V.

---

## Wiring (short)

- **Pi ↔ ESP32:** Connect the ESP32-S3 to the Pi with a **USB cable**. The Pi talks to the ESP32 over this link (no separate UART wiring).
- **ESP32 GPIO 4, 5** → servo signal pins; servos’ V+ and GND to 5 V supply.
- **ESP32 GPIO 6, 7, 8** → L298N IN1, IN2, Enable A; L298N out to DC motor and motor power supply.

### Finding the serial port on the Pi (USB connection)

With the ESP32-S3 plugged into the Pi via USB, run on the Pi:

```bash
ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
```

- ESP32-S3 with built-in USB CDC usually appears as `**/dev/ttyACM0**`.
- A USB-serial adapter may show as `**/dev/ttyUSB0**`.

Set that device in `config/hardware.yaml` under `serial.port`. The default is `/dev/ttyACM0` for a direct USB connection.

---

## Build and upload

### Option A: From the Raspberry Pi (Arduino CLI)

With the ESP32-S3 connected to the Pi via USB, you can compile and flash from the Pi.

**1. Install Arduino CLI and ESP32 support (one-time):**

```bash
# Install Arduino CLI (Linux ARM64 for Pi 5; for Pi 4 use linuxarm)
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
sudo mv bin/arduino-cli /usr/local/bin/

# Add ESP32 board index and install the core (can take several minutes)
arduino-cli config add board_manager.additional_urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32

# Install the ESP32Servo library
arduino-cli lib install "ESP32Servo"
```

**2. Build and upload** (from the **yolo-project** root):

```bash
# Clean build so partitions.bin and other artifacts are generated
arduino-cli compile --clean --fqbn esp32:esp32:esp32s3 esp32/apple_car_rc
arduino-cli board list
arduino-cli upload -p /dev/ttyACM0 --fqbn esp32:esp32:esp32s3 esp32/apple_car_rc
```

Use the port from `board list` (e.g. `/dev/ttyACM0`). To **scan for ports** without building:
```bash
bash esp32/upload_from_pi.sh scan
```
Or run the full script (it auto-detects a USB port if you don’t pass one):

```bash
bash esp32/upload_from_pi.sh
```

---

### Option B: From a PC with Arduino IDE

1. Open **Arduino IDE**.
2. **Board:** Tools → Board → **ESP32 Arduino** → your ESP32-S3 board (e.g. “ESP32S3 Dev Module”).
3. **Port:** Tools → Port → (ESP32’s USB port).
4. **Sketch:** open `apple_car_rc/apple_car_rc.ino`.
5. **Library:** Install **ESP32Servo** (Sketch → Include Library → Manage Libraries → search “ESP32Servo”).
6. **Upload:** Sketch → Upload.

After upload, open **Tools → Serial Monitor** at **115200 baud** to see `apple_car_rc ready; waiting for S D T lines from Pi`.

---

## Pi side

In `config/hardware.yaml` set:

```yaml
interface: serial
serial:
  port: "/dev/ttyACM0"   # or /dev/ttyUSB0, check with ls /dev/tty*
  baud_rate: 115200
```

Then run `python inference.py` (or `python inference.py --web`). The Pi will send one line per frame; the ESP32 will drive the servos and motor.

---

## Troubleshooting

- **`Failed to connect to ESP32-S3: No serial data received`** — Wrong port. With the ESP32 connected to the Pi **via USB**, it shows as **`/dev/ttyACM0`** or **`/dev/ttyUSB0`**, not `/dev/ttyAMA*` (Pi internal UART). Run `arduino-cli board list` with the ESP32 plugged in and use the port listed (e.g. upload with `-p /dev/ttyACM0`). If it still fails, hold the **BOOT** button on the ESP32, start the upload, then release BOOT when you see "Connecting...".

- **`partitions.bin: No such file or directory`** — The build cache didn’t have the right artifacts. Do a **clean build** before upload:
  ```bash
  arduino-cli compile --clean --fqbn esp32:esp32:esp32s3 esp32/apple_car_rc
  ```
  Then run the upload again. The script `upload_from_pi.sh` uses `--clean` by default.

---

## Safety

- If the Pi stops sending for **500 ms**, the firmware stops the motor (timeout). Change `SERIAL_TIMEOUT_MS` in the sketch if you want a different timeout.
- You can reduce max motor speed by lowering `MOTOR_PWM_MAX` (e.g. 200 instead of 255).

