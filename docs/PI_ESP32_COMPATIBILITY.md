# Pi ↔ ESP32 compatibility checklist (first test)

Use this to confirm the Raspberry Pi and ESP32 firmware match before your first run.

---

## 1. Serial protocol

| Item | Pi | ESP32 | Match |
|------|----|-------|-------|
| **Format** | `S <steer> D <drive> T <tilt>\n` | `sscanf(line, "S %f D %f T %f", ...)` | ✅ Same |
| **Order** | S, D, T | steer, drive, tilt | ✅ Same |
| **Values** | -1.0 to +1.0 (float, 3 decimals) | -1.0 to +1.0 (clamped) | ✅ Same |
| **Line ending** | `\n` | Accepts `\n` or `\r` | ✅ Compatible |
| **Baud rate** | 115200 (`config/hardware.yaml`) | 115200 (`SERIAL_BAUD`) | ✅ Same |

**Pi sends (example):** `S 0.120 D 0.450 T -0.050\n`  
**ESP32 parses:** same string; drives steering servo, motor, tilt servo.

---

## 2. Value meaning (both sides)

| Signal | -1.0 | 0.0 | +1.0 |
|--------|------|-----|------|
| **S (steer)** | Full left | Straight | Full right |
| **D (drive)** | Full reverse | Stop | Full forward |
| **T (tilt)** | Full down | Level | Full up |

Pi `ControlOutput` and ESP32 behaviour use this convention consistently.

---

## 3. Pi config (for serial)

In **`config/hardware.yaml`**:

```yaml
interface: serial

serial:
  port: "/dev/ttyACM0"   # or /dev/ttyUSB0 — use: bash esp32/upload_from_pi.sh scan
  baud_rate: 115200
  verbose: true          # echo each S D T line to console (default on)
```

- **Port:** Must be the ESP32’s USB serial port (see `esp32/README.md`).
- **Baud:** Must be 115200 (same as ESP32).

---

## 4. ESP32 hardware (this firmware)

- **Steering:** GPIO 4 → Hitec HS-646WP (1000–2000 µs @ 50 Hz).
- **Tilt:** GPIO 5 → Hitec HS-646WP (1000–2000 µs @ 50 Hz).
- **Motor:** GPIO 6 = IN1, 7 = IN2, 8 = Enable PWM → L298N → DC motor.

Library: **ESP32Servo**. API: **ESP32 Arduino 3.x** (`ledcAttach`/`ledcWrite` by pin).

---

## 5. Pi flow (inference → serial)

1. Camera frame → detector → tracker → controller → **ControlOutput**.
2. **hardware.apply(output)** sends one line per frame: `S ... D ... T ...\n`.
3. With `serial.verbose: true`, that line is also printed to the console.

So every frame, the Pi sends one line; the ESP32 parses it and updates servos and motor. If the Pi stops sending, the ESP32 stops the motor after 500 ms (safety timeout).

---

## 6. Quick test (no car movement)

1. **Pi:** Set `interface: serial`, correct `serial.port`, `verbose: true`. Run:
   ```bash
   cd ~/yolo-project && source .venv/bin/activate
   python inference.py --headless
   ```
2. **Console:** You should see lines like `S 0.000 D 0.000 T 0.000` (or non-zero when an apple is detected). That is what the Pi is sending.
3. **ESP32:** If connected over USB to the Pi, it receives those lines and drives servos/motor. With no apple, S/D/T are 0 → steering centre, motor stop, tilt level.

---

## 7. Common mismatches

| Problem | Check |
|--------|--------|
| ESP32 does nothing | Pi `interface: serial`? Correct `serial.port`? ESP32 on that port? (`bash esp32/upload_from_pi.sh scan`) |
| Wrong direction (e.g. steer left/right flipped) | On ESP32 set `INVERT_STEER 1` (or `INVERT_DRIVE` / `INVERT_TILT`) in the .ino and reflash. |
| Motor runs when it shouldn’t | ESP32 safety timeout (500 ms) only stops motor; servos hold last position. Pi sends 0,0,0 when no apple. |
| No serial output on Pi | `serial.verbose: true` in `config/hardware.yaml`. |

---

## 8. Summary

- **Protocol:** Pi and ESP32 use the same text format and value range; no code change needed for compatibility.
- **Config:** Pi uses `config/hardware.yaml` (interface, port, baud, verbose); ESP32 uses `SERIAL_BAUD` and pin #defines.
- **First test:** Run inference with `interface: serial` and `verbose: true`; confirm S D T lines in the console and ESP32 reacting (servos/motor) when the camera sees an apple.
