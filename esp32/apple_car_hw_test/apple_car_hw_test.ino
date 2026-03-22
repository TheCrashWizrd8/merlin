/*
 * ESP32-S3 Standalone Hardware Test
 *
 * Purpose:
 *   Secondary test firmware for bench testing the apple car electronics
 *   without Raspberry Pi control.
 *
 * Hardware:
 *   - PCA9685 servo driver (I2C SDA=GPIO8, SCL=GPIO9)
 *     Channel 0 = steering servo
 *     Channel 1 = tilt servo
 *   - L298N motor controller
 *     IN1=GPIO4, IN2=GPIO5, PWM/EN=GPIO6
 *
 * Serial monitor commands (115200):
 *   1  -> motor forward test pulse
 *   2  -> motor reverse test pulse
 *   0  -> motor stop
 *   s  -> servo sweep demo (steer + tilt)
 *   m  -> automatic motor cycle demo
 *   h  -> help
 */

// Compatibility shim: some ESP32-S3 toolchain paths don't define ESP32
// macro, but Adafruit_BusIO uses it for SPI type selection.
#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// I2C
#define I2C_SDA 8
#define I2C_SCL 9

// PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);
#define SERVO_STEER_CHANNEL 0
#define SERVO_TILT_CHANNEL  1
#define SERVO_MIN           205
#define SERVO_MAX           410
#define SERVO_CENTER        307
#define PCA9685_OSC_FREQ    25000000

// L298N
#define PIN_MOTOR_IN1 4
#define PIN_MOTOR_IN2 5
#define PIN_MOTOR_PWM 6
#define MOTOR_PWM_FREQ  5000
#define MOTOR_PWM_RES   8
#define MOTOR_MAX_SPEED 255
#define MOTOR_MIN_START 90

static float clampf(float v) {
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

uint16_t servoTick(float v) {
  v = clampf(v);
  float halfRange = (SERVO_MAX - SERVO_MIN) / 2.0f;
  return (uint16_t)(SERVO_CENTER + (v * halfRange));
}

void setServo(float steer, float tilt) {
  pca9685.setPWM(SERVO_STEER_CHANNEL, 0, servoTick(steer));
  pca9685.setPWM(SERVO_TILT_CHANNEL, 0, servoTick(tilt));
}

void setMotor(float drive) {
  drive = clampf(drive);
  int pwm = (int)(fabsf(drive) * MOTOR_MAX_SPEED);
  if (pwm > 0) pwm = max(MOTOR_MIN_START, pwm);
  pwm = min(MOTOR_MAX_SPEED, pwm);

  if (drive > 0.02f) {
    digitalWrite(PIN_MOTOR_IN1, HIGH);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    ledcWrite(PIN_MOTOR_PWM, pwm);
  } else if (drive < -0.02f) {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, HIGH);
    ledcWrite(PIN_MOTOR_PWM, pwm);
  } else {
    digitalWrite(PIN_MOTOR_IN1, LOW);
    digitalWrite(PIN_MOTOR_IN2, LOW);
    ledcWrite(PIN_MOTOR_PWM, 0);
  }
}

void printHelp() {
  Serial.println();
  Serial.println("=== apple_car_hw_test commands ===");
  Serial.println("1 : motor forward pulse");
  Serial.println("2 : motor reverse pulse");
  Serial.println("0 : motor stop");
  Serial.println("s : servo sweep demo");
  Serial.println("m : motor auto cycle demo");
  Serial.println("h : this help");
  Serial.println();
}

void motorPulse(float drive, unsigned long ms) {
  setMotor(drive);
  Serial.print("Motor drive=");
  Serial.println(drive, 2);
  delay(ms);
  setMotor(0.0f);
  Serial.println("Motor stop");
}

void runMotorCycle() {
  Serial.println("Motor cycle: FWD -> STOP -> REV -> STOP");
  motorPulse(+0.40f, 1200);
  delay(400);
  motorPulse(-0.40f, 1200);
  delay(400);
}

void runServoSweep() {
  Serial.println("Servo sweep start");
  for (float x = -1.0f; x <= 1.001f; x += 0.15f) {
    setServo(x, -x * 0.6f);
    delay(130);
  }
  for (float x = 1.0f; x >= -1.001f; x -= 0.15f) {
    setServo(x, -x * 0.6f);
    delay(130);
  }
  setServo(0.0f, 0.0f);
  Serial.println("Servo sweep done");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);

  pca9685.begin();
  pca9685.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pca9685.setPWMFreq(50);
  delay(10);
  setServo(0.0f, 0.0f);

  pinMode(PIN_MOTOR_IN1, OUTPUT);
  pinMode(PIN_MOTOR_IN2, OUTPUT);
  digitalWrite(PIN_MOTOR_IN1, LOW);
  digitalWrite(PIN_MOTOR_IN2, LOW);
  ledcAttach(PIN_MOTOR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_MOTOR_PWM, 0);

  Serial.println("apple_car_hw_test ready");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  char c = (char)Serial.read();
  if (c == '\r' || c == '\n') return;

  switch (c) {
    case '1':
      motorPulse(+0.40f, 1200);
      break;
    case '2':
      motorPulse(-0.40f, 1200);
      break;
    case '0':
      setMotor(0.0f);
      Serial.println("Motor stop");
      break;
    case 's':
      runServoSweep();
      break;
    case 'm':
      runMotorCycle();
      break;
    case 'h':
      printHelp();
      break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(c);
      printHelp();
      break;
  }
}
