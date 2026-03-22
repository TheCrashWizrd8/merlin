# Pi → ESP32 Pro S3 serial protocol

The Raspberry Pi runs inference and sends **one line per frame** over USB serial to an **ESP32 Pro S3**. The ESP32 drives:

- **Steering** — Hitec HS-646WP servo
- **Camera tilt** — Hitec HS-646WP servo  
- **Drive** — DC motor via **L298N** (direction + PWM speed)

---

## Architecture

```
┌─────────────────┐     USB serial      ┌─────────────────────┐
│  Raspberry Pi   │ ──────────────────► │  ESP32 Pro S3       │
│  inference.py   │   S D T values      │  • 2× servo PWM     │
│  hardware:      │   (text, 115200)    │ • L298N IN1,IN2,PWM │
│  interface:     │                     └──────────┬──────────┘
│  serial         │                                │
└─────────────────┘                                ▼
                                          ┌───────────────────┐
                                          │ L298N motor driver│
                                          │ → DC drive motor  │
                                          └───────────────────┘
```

---

## Serial protocol

- **Port:** set in `config/hardware.yaml` under `serial.port` (e.g. `/dev/ttyUSB0` when ESP32 is connected over USB).
- **Baud rate:** `serial.baud_rate` (default `115200`).
- **Format:** one line per frame, ASCII:

  ```
  S <steer> D <drive> T <tilt>\n
  ```

  Each value is a float from **-1.0** to **+1.0**, e.g.:

  ```
  S 0.120 D 0.450 T -0.050
  S -0.300 D 0.000 T 0.000
  ```

| Letter | Meaning           | -1.0        | 0.0   | +1.0       |
|--------|-------------------|-------------|-------|------------|
| **S**  | Steering servo    | full left   | straight | full right |
| **D**  | Drive motor       | full reverse | stop  | full forward |
| **T**  | Camera tilt servo | full down   | level | full up    |

**Semantics:**
- **S, T** = *angle* — servos move to and hold that position (0 = centre).
- **D** = *sustained speed* — the motor runs at that speed until a different D value is sent. D=1 means full forward continuously; D=0 means stop. Unlike servos, D is not a position — it is an ongoing speed command.

The Pi sends at inference frame rate (e.g. 10–25 Hz). The ESP32 should parse each line and update outputs; if no line is received for a while, you may want to stop the motor (e.g. D=0) for safety.

---

## ESP32 side: what to implement

### 1. Serial

- Open the same baud rate (e.g. 115200).
- Read a line (up to `\n`), then parse: `S <float> D <float> T <float>`.
- You can use `sscanf` or split on spaces and match the letters.

### 2. Steering and tilt servos (Hitec HS-646WP)

- Use two PWM outputs (e.g. ESP32 LEDC) at **50 Hz**.
- Map value **-1.0 → 1.0** to pulse width **1000 µs → 2000 µs** (1500 µs = centre).
- Example (concept):  
  `pulse_us = 1500 + (value * 500)` then clamp to 1000–2000.

### 3. Drive motor via L298N

- **L298N** has IN1, IN2 and (optionally) enable/PWM.
- **Direction:**  
  - D &gt; 0: forward  → e.g. IN1=HIGH, IN2=LOW  
  - D &lt; 0: reverse → e.g. IN1=LOW, IN2=HIGH  
  - D = 0: stop      → IN1=LOW, IN2=LOW (or both LOW)
- **Speed:** map `abs(D)` (0.0–1.0) to your PWM duty (0–255 or 0–1023). Use the same enable/PWM pin for both directions.
- Optionally cap max duty (e.g. 80%) for safety.

Example (pseudo):

- `speed = (uint8_t)(fminf(1.0f, fabsf(drive)) * 255.0f);`
- if `drive > 0`: IN1=1, IN2=0, PWM=speed  
- if `drive < 0`: IN1=0, IN2=1, PWM=speed  
- if `drive == 0`: IN1=0, IN2=0, PWM=0  

---

## Pi configuration

In `config/hardware.yaml`:

```yaml
interface: serial

serial:
  port: "/dev/ttyUSB0"   # or the device the ESP32 gets (check with ls /dev/tty*)
  baud_rate: 115200
```

Install: `pip install pyserial`

Connect the ESP32 over USB to the Pi; then run `python inference.py` (or `python inference.py --web`). The Pi will send `S D T` lines every frame; the ESP32 handles servos and L298N.

**Firmware:** Arduino sketch for ESP32-S3 is in **`esp32/apple_car_rc/`**; see **`esp32/README.md`** for pinout, wiring, and upload.
