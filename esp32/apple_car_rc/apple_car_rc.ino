/*
 * ESP32-S3 RC Receiver
 *
 * Hardware:
 *  - PCA9685 servo driver (I2C, SDA=GPIO8, SCL=GPIO9)
 *    Channel 0 = steering servo (S)
 *    Channel 1 = tilt servo (T)
 *  - L298N motor controller (D)
 *    IN1=GPIO4, IN2=GPIO5, PWM/EN=GPIO6
 *
 * Serial input from Raspberry Pi:
 *   S <steer> D <drive> T <tilt>
 *   Values: -1.0 to +1.0
 *
 * Semantics:
 *   S, T = angle (servos hold position). 0 = centre, -1 = full left/down, +1 = full right/up.
 *   D    = sustained speed (engine). D=1 means run at full speed until a different D is sent.
 *         The motor keeps its last commanded speed; only a new D value or timeout changes it.
 */

// Compatibility shim: some ESP32-S3 toolchain paths don't define ESP32
// macro, but Adafruit_BusIO uses it for SPI type selection.
#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// -----------------------------------------------------
// I2C pins (ESP32-S3)
// -----------------------------------------------------

#define I2C_SDA 8
#define I2C_SCL 9

// -----------------------------------------------------
// PCA9685 servo driver
// -----------------------------------------------------

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

#define SERVO_STEER_CHANNEL 0
#define SERVO_TILT_CHANNEL  1

// PCA9685 at 50 Hz, 12-bit (4096 steps), period = 20 ms
// 1 step = 20000 µs / 4096 = 4.8828 µs
// 1000 µs = 204.8 → 205   (full left / full down)
// 1500 µs = 307.2 → 307   (centre / level)
// 2000 µs = 409.6 → 410   (full right / full up)
#define SERVO_MIN    205
#define SERVO_MAX    410
#define SERVO_CENTER 307

// PCA9685 crystal oscillator frequency (Hz)
// 25 MHz is standard for Adafruit and most clones
#define PCA9685_OSC_FREQ 25000000

// -----------------------------------------------------
// L298N motor controller pins
// -----------------------------------------------------

#define PIN_MOTOR_IN1 4
#define PIN_MOTOR_IN2 5
#define PIN_MOTOR_PWM 6

#define MOTOR_PWM_FREQ  5000
#define MOTOR_PWM_RES   8
#define MOTOR_MAX_SPEED 255
#define MOTOR_DEADBAND  0.02f
#define MOTOR_MIN_START 90

// -----------------------------------------------------
// Serial config
// -----------------------------------------------------

#define SERIAL_BAUD       115200
#define SERIAL_TIMEOUT_MS 8000   // Safety only: stop motor if Pi silent this long (D sustains until new value)

unsigned long lastSerial = 0;
float lastDrive = 0.0f;   // D sustains: motor keeps this until new line or timeout

// -----------------------------------------------------

float clampf(float v)
{
  if (v < -1.0f) return -1.0f;
  if (v >  1.0f) return  1.0f;
  return v;
}

// -----------------------------------------------------
// Convert -1.0..+1.0 → PCA9685 tick count
// -1.0 → SERVO_MIN (1000 µs), 0.0 → SERVO_CENTER (1500 µs), +1.0 → SERVO_MAX (2000 µs)
// -----------------------------------------------------

uint16_t servoTick(float v)
{
  v = clampf(v);
  float halfRange = (SERVO_MAX - SERVO_MIN) / 2.0f;
  return (uint16_t)(SERVO_CENTER + (v * halfRange));
}

// -----------------------------------------------------
// Motor control via L298N — D is sustained speed, not a pulse.
// Apply v and keep PWM until a new value or timeout.
// -----------------------------------------------------

void setMotor(float v)
{
  v = clampf(v);
  float mag = fabsf(v);
  int motorSpeed = (int)(mag * MOTOR_MAX_SPEED);
  if (motorSpeed > 0) {
    // Help overcome motor + gearbox static friction at low commands.
    motorSpeed = max(MOTOR_MIN_START, motorSpeed);
  }
  motorSpeed = min(MOTOR_MAX_SPEED, motorSpeed);

  if (v > MOTOR_DEADBAND)
  {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    ledcWrite(PIN_MOTOR_PWM, motorSpeed);
  }
  else if (v < -MOTOR_DEADBAND)
  {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    ledcWrite(PIN_MOTOR_PWM, motorSpeed);
  }
  else
  {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    ledcWrite(PIN_MOTOR_PWM, 0);
  }
}

// -----------------------------------------------------
// Serial parser
// -----------------------------------------------------

void parseLine(char *line)
{
  float steer, drive, tilt;

  if (sscanf(line, "S %f D %f T %f", &steer, &drive, &tilt) != 3)
    return;

  lastSerial = millis();
  lastDrive = drive;   // D sustains until next line or timeout

  pca9685.setPWM(SERVO_STEER_CHANNEL, 0, servoTick(steer));
  pca9685.setPWM(SERVO_TILT_CHANNEL,  0, servoTick(tilt));
  setMotor(drive);

  Serial.print("OK S:");
  Serial.print(steer, 3);
  Serial.print(" D:");
  Serial.print(drive, 3);
  Serial.print(" T:");
  Serial.print(tilt, 3);
  Serial.print(" -> steer_tick:");
  Serial.print(servoTick(steer));
  Serial.print(" tilt_tick:");
  Serial.print(servoTick(tilt));
  Serial.print(" motor_pwm:");
  int dbgPwm = (int)(fabsf(clampf(drive)) * MOTOR_MAX_SPEED);
  if (dbgPwm > 0) dbgPwm = max(MOTOR_MIN_START, dbgPwm);
  dbgPwm = min(MOTOR_MAX_SPEED, dbgPwm);
  if (fabsf(drive) <= MOTOR_DEADBAND) dbgPwm = 0;
  Serial.print(dbgPwm);
  Serial.print(" dir:");
  if (drive > MOTOR_DEADBAND) Serial.println("FWD");
  else if (drive < -MOTOR_DEADBAND) Serial.println("REV");
  else Serial.println("STOP");
}

// -----------------------------------------------------

#define RX_BUF 80

char rxBuf[RX_BUF];
int  rxPos = 0;

// -----------------------------------------------------

void setup()
{
  Serial.begin(SERIAL_BAUD);
  delay(200);

  // I2C with explicit pins for ESP32-S3
  Wire.begin(I2C_SDA, I2C_SCL);

  // Initialise PCA9685
  pca9685.begin();
  pca9685.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pca9685.setPWMFreq(50);
  delay(10);

  // Centre both servos on boot
  pca9685.setPWM(SERVO_STEER_CHANNEL, 0, SERVO_CENTER);
  pca9685.setPWM(SERVO_TILT_CHANNEL,  0, SERVO_CENTER);

  // L298N motor pins
  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);

  ledcAttach(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_MOTOR_PWM, 0);

  lastSerial = millis();

  Serial.println("RC receiver ready  (SDA=" + String(I2C_SDA) + " SCL=" + String(I2C_SCL) + ")");
  Serial.println("Servos centred. Waiting for S D T lines...");
}

// -----------------------------------------------------

void loop()
{
  while (Serial.available())
  {
    char c = Serial.read();

    if (c == '\n' || c == '\r')
    {
      if (rxPos > 0)
      {
        rxBuf[rxPos] = '\0';
        parseLine(rxBuf);
        rxPos = 0;
      }
    }
    else if (rxPos < RX_BUF - 1)
    {
      rxBuf[rxPos++] = c;
    }
    else
    {
      rxPos = 0;
    }
  }

  // Safety: only stop motor when Pi has been silent — D sustains until then
  if (millis() - lastSerial > SERIAL_TIMEOUT_MS)
  {
    if (lastDrive != 0.0f)
    {
      lastDrive = 0.0f;
      setMotor(0);
    }
  }
  else
  {
    // Re-apply lastDrive every loop so motor keeps running at speed (not pulse-like).
    // Servos hold position; motor needs sustained PWM to sustain speed.
    setMotor(lastDrive);
  }
}
 