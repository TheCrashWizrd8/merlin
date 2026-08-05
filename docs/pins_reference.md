# Pins Reference (Car + Sub)

This document is a human-readable companion to `config/pins.yaml`.

It captures the current known ESP32/PCA9685/motor pinout (car) and marks the sub wiring pieces as `TBD` until you confirm the actual channel/pin numbers.

## Car (current firmware) wiring

### ESP32-S3 -> PCA9685 (I2C)

- `ESP32 SDA` (`I2C_SDA`) -> PCA9685 `SDA` = **GPIO 8**
- `ESP32 SCL` (`I2C_SCL`) -> PCA9685 `SCL` = **GPIO 9**
- PCA9685 I2C address (`Adafruit_PWMServoDriver(0x40)`) = **0x40**
- PCA9685 PWM frequency (`setPWMFreq(50)`) = **50 Hz**

### PCA9685 channel mapping (car servos)

- PCA9685 channel **0** (`SERVO_STEER_CHANNEL`) = `steering_servo` (car steering, labeled `S` in serial)
- PCA9685 channel **1** (`SERVO_TILT_CHANNEL`) = `camera_tilt_servo` (car camera tilt, labeled `T` in serial)

### PCA9685 tick calibration (used by the firmware)

At 50 Hz, the sketch uses:

- `SERVO_MIN` tick = **205** (approx **1000 us**)
- `SERVO_CENTER` tick = **307** (approx **1500 us**)
- `SERVO_MAX` tick = **410** (approx **2000 us**)

### ESP32 -> L298N motor driver (car drive)

Car drive uses the L298N pins:

- `IN1` = **GPIO 4**
- `IN2` = **GPIO 5**
- PWM/EN = **GPIO 6**

The firmware drives direction using IN1/IN2 and sustained speed using PWM based on the `D` value.

## Raspberry Pi -> ESP32 serial protocol (car)

Pi sends one line per update in ASCII:

- `S <steer> D <drive> T <tilt>\n`

Ranges (as used by the Python + ESP32 firmware):

- `steer`: `-1.0 .. +1.0` (servo holds at angle)
- `drive`: `-1.0 .. +1.0` (motor sustained speed; `D` sustains until a new line or timeout)
- `tilt`: `-1.0 .. +1.0` (servo holds position)

Safety:

- if Pi stops sending for `SERIAL_TIMEOUT_MS` = **8000 ms**, motor stops (servos hold).

## Sub (unconfirmed) wiring checklist

Fill these in once you confirm the actual PCA9685 channel numbers and thruster driver pins.

### PCA9685 channels (sub servos)

- `aftSteerY` uses PCA9685 channel: **TBD**
- `aftSteerZ` uses PCA9685 channel: **TBD**
- `finLeft` uses PCA9685 channel: **TBD**
- `finRight` uses PCA9685 channel: **TBD**

### Thruster driver pins (sub motor)

The current car firmware uses L298N pins `IN1/IN2/PWM` on GPIO `4/5/6`.

For the sub thruster, the mapping is currently:

- `thruster in1_gpio`: **TBD**
- `thruster in2_gpio`: **TBD**
- `thruster pwm_gpio`: **TBD**

If your sub thruster uses the same physical L298N wiring as the car motor, you can set these to the same GPIOs as the car drive (`4/5/6`). Otherwise, provide the correct GPIOs.

### Proposed sub serial format (for firmware implementation)

Proposed new line format (to be implemented in ESP32 firmware):

- `S2 <aftSteerY> <aftSteerZ> F <finLeft> <finRight> X <thrusterX>\n`

Once the actual pins/channels are confirmed, update the firmware parsing + Pi side mapping accordingly.

