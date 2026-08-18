// ESP32-S3 — Sub vehicle receiver + sensor hub
// Pi link: USB Serial (default) or GPIO UART — see USE_PI_UART below.
//
// PINOUT — edit these to match your wiring (-1 = not connected)

// GPIO UART (optional): cross Pi GPIO14 TX / GPIO15 RX with ESP RX/TX
#define USE_PI_UART           0
#define PI_UART_RX            44    // ESP RX <- Pi TX (header pin 8)
#define PI_UART_TX            43    // ESP TX -> Pi RX (header pin 10)

// I2C bus — shared by PCA9685 (servos) and MPU6050 (GY-521 IMU)
// GPIO 29/30 are internal flash on most ESP32-S3 modules (boot loop if used for I2C).
// Wire SDA/SCL to GPIO 8/9 (or 21/22) instead.
#define I2C_SDA               8
#define I2C_SCL               9
#define PCA9685_ADDR          0x40
#define ENABLE_PCA9685        1
#define MPU6050_ADDR          0x68    // GY-521 default (AD0 → GND)
#define ENABLE_MPU6050        1

// PCA9685 channels: aft steer Y/Z, fore fins L/R, sonar sweep servo
#define CH_AFT_STEER_Y        0
#define CH_AFT_STEER_Z        1
#define CH_FIN_LEFT           2
#define CH_FIN_RIGHT          3
#define CH_SONAR_SERVO        4

// Sonar — HC-SR04 / JSN-SR04T style (trigger + echo). Set ENABLE_SONAR=0 to disable.
#define ENABLE_SONAR          1
#define PIN_SONAR_TRIG        15
#define PIN_SONAR_ECHO        16
#define SONAR_MAX_RANGE_M     6.0f
#define SONAR_STEP_DEG        5
#define SONAR_SWEEP_MIN       -180
#define SONAR_SWEEP_MAX       180
#define SONAR_SETTLE_MS       25
#define SONAR_PULSE_TIMEOUT_US 35000

// Thruster via L298N (IN2 moved — GPIO 5 is leak sensor)
#define PIN_THR_IN1           4
#define PIN_THR_IN2           12
#define PIN_THR_PWM           6

// Ballast — Makerverse Motor Driver 2 Channel (DIR/PWM mode, on/off fill/drain)
// Fore: channel A (DIR A + PWM A). Aft: channel B (DIR B + PWM B).
// Pot wipers → ESP32 ADC (3.3V, wiper, GND on each linear pot)
// GPIO 7 = ADC1 (aft), GPIO 11 = ADC2 (fore — WiFi must stay off)
#define BALLAST_USE_DIR_PWM   1
#define PIN_FORE_BALLAST_DIR  13    // DIR A
#define PIN_FORE_BALLAST_PWM  14    // PWM A
#define PIN_FORE_BALLAST_POT  11
#define PIN_AFT_BALLAST_DIR   8     // DIR B
#define PIN_AFT_BALLAST_PWM   9     // PWM B
#define PIN_AFT_BALLAST_POT   7

// Sensors
#define PIN_BATTERY_ADC       1     // ADC1
#define PIN_DEPTH_ADC         3     // ADC1 (or I2C depth sensor later)
#define DEPTH_I2C_ADDR        -1    // TODO: I2C depth sensor address when wired

// Leak detector — Blue Robotics SOS (active HIGH = leak). Set to -1 when unwired.
#define PIN_LEAK              5

// Timing
#define SERIAL_BAUD           115200
#define SERIAL_TIMEOUT_MS     8000
#define TELEMETRY_HZ          5
#define MOTOR_PWM_FREQ        5000
#define MOTOR_PWM_RES         8
#define MOTOR_MAX_SPEED       255
#define MOTOR_DEADBAND        0.02f
#define MOTOR_MIN_START       90
#define BALLAST_CMD_DEADBAND  0.05f

// Servo PWM ticks @ 50 Hz (PCA9685 12-bit, 25 MHz osc)
#define SERVO_MIN             205
#define SERVO_MAX             410
#define SERVO_CENTER          307
#define PCA9685_OSC_FREQ      25000000

#define BALLAST_FORE          0
#define BALLAST_AFT           1
#define BALLAST_COUNT         2

// Mirror key messages to USB Serial for post-flash debugging on Mac/PC.
#define USB_DEBUG_MIRROR      1

#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Preferences.h>
#include <math.h>
#if ENABLE_PCA9685 || ENABLE_MPU6050
#include <Wire.h>
#endif
#if ENABLE_PCA9685
#include <Adafruit_PWMServoDriver.h>
#endif
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#endif

#if USE_PI_UART
HardwareSerial PiLink(1);
#define LINK PiLink
#else
#define LINK Serial
#endif

#if ENABLE_PCA9685
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_ADDR);
#endif

struct BallastState {
  int pinIna;
  int pinInb;
  int pinPot;
  float command;
  float pos;
  int adc;
  bool moving;
  const char *dir;
  int adcTop;
  int adcBottom;
  bool calValid;
  const char *name;
  const char *nvsTopKey;
  const char *nvsBotKey;
  const char *nvsCalKey;
};

struct HardwareState {
  float batteryV;
  float pitch, roll, yaw;
  float depthM;
  bool leak[4];
  float aftY, aftZ, finL, finR, thruster;
  int thrusterPwm;
  const char *status;
  const char *fault;
  unsigned long heartbeat;
};

HardwareState hw = {};

BallastState ballasts[BALLAST_COUNT] = {
  { PIN_FORE_BALLAST_DIR, PIN_FORE_BALLAST_PWM, PIN_FORE_BALLAST_POT,
    0.0f, 0.0f, 0, false, "STOP", -1, -1, false, "fore", "fTop", "fBot", "fCal" },
  { PIN_AFT_BALLAST_DIR, PIN_AFT_BALLAST_PWM, PIN_AFT_BALLAST_POT,
    0.0f, 0.0f, 0, false, "STOP", -1, -1, false, "aft", "aTop", "aBot", "aCal" },
};

unsigned long lastSerial = 0;
unsigned long lastTelem = 0;
float lastThruster = 0.0f;
bool testMode = false;
unsigned long testModeUntil = 0;
bool pca9685Ok = false;
bool mpu6050Ok = false;
bool piUartReady = false;
Preferences ballastPrefs;

#define RX_BUF 120
char rxBuf[RX_BUF];
int rxPos = 0;

static float clampf(float v) {
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static void sayLine(const char *msg) {
#if USE_PI_UART && USB_DEBUG_MIRROR
  Serial.println(msg);
#endif
#if USE_PI_UART
  if (piUartReady) LINK.println(msg);
#else
  LINK.println(msg);
#endif
}

static void sayLine(const String &msg) {
#if USE_PI_UART && USB_DEBUG_MIRROR
  Serial.println(msg);
#endif
#if USE_PI_UART
  if (piUartReady) LINK.println(msg);
#else
  LINK.println(msg);
#endif
}

static void bootCheckpoint(const char *tag) {
  Serial.println(tag);
#if USE_PI_UART
  if (piUartReady) LINK.println(tag);
#else
  LINK.println(tag);
#endif
}

static uint16_t servoTick(float v) {
  v = clampf(v);
  float half = (SERVO_MAX - SERVO_MIN) / 2.0f;
  return (uint16_t)(SERVO_CENTER + (v * half));
}

static void setServoChannel(int ch, float v) {
#if ENABLE_PCA9685
  if (!pca9685Ok || ch < 0 || ch > 4) return;
  pca9685.setPWM(ch, 0, servoTick(v));
#else
  (void)ch;
  (void)v;
#endif
}

static void setSonarServoDeg(int angleDeg) {
  if (angleDeg < SONAR_SWEEP_MIN) angleDeg = SONAR_SWEEP_MIN;
  if (angleDeg > SONAR_SWEEP_MAX) angleDeg = SONAR_SWEEP_MAX;
  setServoChannel(CH_SONAR_SERVO, (float)angleDeg / 180.0f);
}

#if ENABLE_SONAR
static float readSonarRangeM() {
  if (PIN_SONAR_TRIG < 0 || PIN_SONAR_ECHO < 0) return -1.0f;
  digitalWrite(PIN_SONAR_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_SONAR_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_SONAR_TRIG, LOW);
  unsigned long duration = pulseIn(PIN_SONAR_ECHO, HIGH, SONAR_PULSE_TIMEOUT_US);
  if (duration == 0) return -1.0f;
  float meters = (duration * 0.0343f / 2.0f) / 100.0f;
  if (meters <= 0.0f || meters > SONAR_MAX_RANGE_M) return -1.0f;
  return meters;
}

enum SonarPhase { SONAR_MOVING, SONAR_SETTLE, SONAR_READING };
static int sonarAngle = SONAR_SWEEP_MAX;
static int sonarDir = -1;
static SonarPhase sonarPhase = SONAR_MOVING;
static unsigned long sonarPhaseTs = 0;

static void updateSonarSweep() {
  if (!pca9685Ok || PIN_SONAR_TRIG < 0 || PIN_SONAR_ECHO < 0) return;
  if (testMode) return;

  unsigned long now = millis();
  switch (sonarPhase) {
    case SONAR_MOVING:
      setSonarServoDeg(sonarAngle);
      sonarPhase = SONAR_SETTLE;
      sonarPhaseTs = now;
      break;
    case SONAR_SETTLE:
      if (now - sonarPhaseTs < SONAR_SETTLE_MS) return;
      sonarPhase = SONAR_READING;
      break;
    case SONAR_READING: {
      float range = readSonarRangeM();
      LINK.print("TEL sonarpt ");
      LINK.print(sonarAngle);
      LINK.print(" ");
      LINK.println(range, 2);

      sonarAngle += sonarDir * SONAR_STEP_DEG;
      if (sonarAngle <= SONAR_SWEEP_MIN) {
        sonarAngle = SONAR_SWEEP_MIN;
        sonarDir = 1;
        LINK.println("TEL sonar sweep");
      } else if (sonarAngle >= SONAR_SWEEP_MAX) {
        sonarAngle = SONAR_SWEEP_MAX;
        sonarDir = -1;
        LINK.println("TEL sonar sweep");
      }
      sonarPhase = SONAR_MOVING;
      break;
    }
  }
}
#endif

static void setThruster(float v) {
  v = clampf(v);
  lastThruster = v;
  hw.thruster = v;
  float mag = fabsf(v);
  int pwm = (int)(mag * MOTOR_MAX_SPEED);
  if (pwm > 0) pwm = max(MOTOR_MIN_START, pwm);
  pwm = min(MOTOR_MAX_SPEED, pwm);
  hw.thrusterPwm = (fabsf(v) <= MOTOR_DEADBAND) ? 0 : pwm;

#if PIN_THR_PWM >= 0
  if (v > MOTOR_DEADBAND) {
    digitalWrite(PIN_THR_IN1, HIGH);
    digitalWrite(PIN_THR_IN2, LOW);
    ledcWrite(PIN_THR_PWM, pwm);
  } else if (v < -MOTOR_DEADBAND) {
    digitalWrite(PIN_THR_IN1, LOW);
    digitalWrite(PIN_THR_IN2, HIGH);
    ledcWrite(PIN_THR_PWM, pwm);
  } else {
    digitalWrite(PIN_THR_IN1, LOW);
    digitalWrite(PIN_THR_IN2, LOW);
    ledcWrite(PIN_THR_PWM, 0);
    hw.thrusterPwm = 0;
  }
#else
  (void)pwm;
#endif
}

static float adcRawToVolts(int adc) {
  if (adc < 0) return -1.0f;
  return (adc / 4095.0f) * 3.3f;
}

static int readBallastAdc(int pin) {
  if (pin < 0) return -1;
  // Average a few samples — pots are high-impedance analog inputs.
  long sum = 0;
  for (int i = 0; i < 4; i++) {
    sum += analogRead(pin);
    delayMicroseconds(200);
  }
  return (int)(sum / 4);
}

static void updateBallastAdc(BallastState *t) {
  if (t->pinPot < 0) {
    t->adc = -1;
    t->pos = -1.0f;
    return;
  }
  t->adc = readBallastAdc(t->pinPot);
  t->pos = ballastPosFromAdc(t, t->adc);
}

static float ballastPosFromAdc(BallastState *tank, int adc) {
  if (!tank->calValid || tank->adcTop < 0 || tank->adcBottom < 0) {
    return adc / 4095.0f;
  }
  float span = (float)(tank->adcTop - tank->adcBottom);
  if (fabsf(span) < 50.0f) {
    return adc / 4095.0f;
  }
  return clampf((adc - tank->adcBottom) / span);
}

static void loadBallastCal() {
  ballastPrefs.begin("sub_rc", true);
  for (int i = 0; i < BALLAST_COUNT; i++) {
    BallastState *t = &ballasts[i];
    t->adcTop = ballastPrefs.getInt(t->nvsTopKey, -1);
    t->adcBottom = ballastPrefs.getInt(t->nvsBotKey, -1);
    t->calValid = ballastPrefs.getBool(t->nvsCalKey, false);
  }
  ballastPrefs.end();
}

static void saveBallastCal(BallastState *tank) {
  ballastPrefs.begin("sub_rc", false);
  ballastPrefs.putInt(tank->nvsTopKey, tank->adcTop);
  ballastPrefs.putInt(tank->nvsBotKey, tank->adcBottom);
  ballastPrefs.putBool(tank->nvsCalKey, tank->calValid);
  ballastPrefs.end();
}

static int ballastIndexFromName(const char *name) {
  if (strcmp(name, "fore") == 0 || strcmp(name, "f") == 0) return BALLAST_FORE;
  if (strcmp(name, "aft") == 0 || strcmp(name, "a") == 0) return BALLAST_AFT;
  return -1;
}

static void calibrateBallastTop(BallastState *tank) {
  if (tank->pinPot < 0) {
    LINK.print("ERR CAL B "); LINK.print(tank->name); LINK.println(" pot not wired");
    return;
  }
  tank->adcTop = readBallastAdc(tank->pinPot);
  if (tank->adcBottom >= 0 && abs(tank->adcTop - tank->adcBottom) >= 50) {
    tank->calValid = true;
  }
  saveBallastCal(tank);
  LINK.print("OK CAL B ");
  LINK.print(tank->name);
  LINK.print(" top ");
  LINK.println(tank->adcTop);
}

static void calibrateBallastBottom(BallastState *tank) {
  if (tank->pinPot < 0) {
    LINK.print("ERR CAL B "); LINK.print(tank->name); LINK.println(" pot not wired");
    return;
  }
  tank->adcBottom = readBallastAdc(tank->pinPot);
  if (tank->adcTop >= 0 && abs(tank->adcTop - tank->adcBottom) >= 50) {
    tank->calValid = true;
  }
  saveBallastCal(tank);
  LINK.print("OK CAL B ");
  LINK.print(tank->name);
  LINK.print(" bottom ");
  LINK.println(tank->adcBottom);
}

static void setBallastTank(BallastState *tank, float cmd) {
  tank->command = clampf(cmd);
  bool fill = tank->command > BALLAST_CMD_DEADBAND;
  bool drain = tank->command < -BALLAST_CMD_DEADBAND;

#if BALLAST_USE_DIR_PWM
  // Makerverse DIR/PWM: PWM enables motor; DIR sets direction.
  if (fill) {
    digitalWrite(tank->pinIna, HIGH);
    digitalWrite(tank->pinInb, HIGH);
  } else if (drain) {
    digitalWrite(tank->pinIna, LOW);
    digitalWrite(tank->pinInb, HIGH);
  } else {
    digitalWrite(tank->pinIna, LOW);
    digitalWrite(tank->pinInb, LOW);
  }
#else
  if (fill) {
    digitalWrite(tank->pinIna, HIGH);
    digitalWrite(tank->pinInb, LOW);
  } else if (drain) {
    digitalWrite(tank->pinIna, LOW);
    digitalWrite(tank->pinInb, HIGH);
  } else {
    digitalWrite(tank->pinIna, LOW);
    digitalWrite(tank->pinInb, LOW);
  }
#endif

  tank->moving = fill || drain;
  if (fill) tank->dir = "FILL";
  else if (drain) tank->dir = "DRAIN";
  else tank->dir = "STOP";
}

static void setBallastBoth(float foreCmd, float aftCmd) {
  setBallastTank(&ballasts[BALLAST_FORE], foreCmd);
  setBallastTank(&ballasts[BALLAST_AFT], aftCmd);
}

static void stopAllBallast() {
  setBallastBoth(0.0f, 0.0f);
}

static void setActuators(float aftY, float aftZ, float finL, float finR, float thr) {
  hw.aftY = clampf(aftY);
  hw.aftZ = clampf(aftZ);
  hw.finL = clampf(finL);
  hw.finR = clampf(finR);
  setServoChannel(CH_AFT_STEER_Y, hw.aftY);
  setServoChannel(CH_AFT_STEER_Z, hw.aftZ);
  setServoChannel(CH_FIN_LEFT, hw.finL);
  setServoChannel(CH_FIN_RIGHT, hw.finR);
  setThruster(thr);
}

static float readAdcVolts(int pin) {
  int raw = analogRead(pin);
  return (raw / 4095.0f) * 3.3f;
}

static bool readLeakActive() {
#if PIN_LEAK < 0
  return false;
#else
  return digitalRead(PIN_LEAK) == HIGH;
#endif
}

#if ENABLE_MPU6050
#define MPU6050_WHO_AM_I      0x75
#define MPU6050_WHO_AM_I_VAL  0x68
#define MPU6050_PWR_MGMT_1    0x6B
#define MPU6050_GYRO_CONFIG   0x1B
#define MPU6050_ACCEL_CONFIG  0x1C
#define MPU6050_ACCEL_XOUT_H  0x3B
#define MPU6050_ACCEL_SCALE   8192.0f   // ±4 g
#define MPU6050_GYRO_SCALE    65.5f     // ±500 °/s

static uint8_t mpu6050Addr = MPU6050_ADDR;

static void i2cBegin() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // 100 kHz — reliable on shared bus / longer wires
}

static bool mpu6050WriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpu6050ReadByte(uint8_t addr, uint8_t reg, uint8_t *val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(addr, (uint8_t)1) != 1) return false;
  *val = Wire.read();
  return true;
}

static bool mpu6050Probe(uint8_t addr) {
  uint8_t who = 0;
  return mpu6050ReadByte(addr, MPU6050_WHO_AM_I, &who) && who == MPU6050_WHO_AM_I_VAL;
}

static bool mpu6050Init() {
  const uint8_t addrs[] = { MPU6050_ADDR, (uint8_t)(MPU6050_ADDR | 1) };
  mpu6050Addr = MPU6050_ADDR;
  for (uint8_t i = 0; i < sizeof(addrs); i++) {
    if (!mpu6050Probe(addrs[i])) continue;
    mpu6050Addr = addrs[i];
    if (!mpu6050WriteByte(mpu6050Addr, MPU6050_PWR_MGMT_1, 0x00)) return false;
    delay(10);
    mpu6050WriteByte(mpu6050Addr, MPU6050_GYRO_CONFIG, 0x08);
    mpu6050WriteByte(mpu6050Addr, MPU6050_ACCEL_CONFIG, 0x08);
    return true;
  }
  return false;
}

static bool mpu6050ReadRaw(int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz) {
  Wire.beginTransmission(mpu6050Addr);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(mpu6050Addr, (uint8_t)14) != 14) return false;

  *ax = (int16_t)((Wire.read() << 8) | Wire.read());
  *ay = (int16_t)((Wire.read() << 8) | Wire.read());
  *az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();  // skip temperature
  *gx = (int16_t)((Wire.read() << 8) | Wire.read());
  *gy = (int16_t)((Wire.read() << 8) | Wire.read());
  *gz = (int16_t)((Wire.read() << 8) | Wire.read());
  return true;
}

static void readMpu6050(float *pitchDeg, float *rollDeg, float *yawDeg) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpu6050ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
    *pitchDeg = 0.0f;
    *rollDeg = 0.0f;
    *yawDeg = 0.0f;
    return;
  }

  float axG = ax / MPU6050_ACCEL_SCALE;
  float ayG = ay / MPU6050_ACCEL_SCALE;
  float azG = az / MPU6050_ACCEL_SCALE;
  *pitchDeg = atan2f(ayG, sqrtf(axG * axG + azG * azG)) * 57.2957795f;
  *rollDeg = atan2f(-axG, azG) * 57.2957795f;
  // No magnetometer — yaw is not meaningful; report 0 for dashboard compatibility.
  (void)gx; (void)gy; (void)gz;
  *yawDeg = 0.0f;
}
#endif

static void readSensors() {
  hw.batteryV = readAdcVolts(PIN_BATTERY_ADC) * 4.0f;
  hw.depthM = readAdcVolts(PIN_DEPTH_ADC);
  hw.leak[0] = readLeakActive();
  hw.leak[1] = false;
  hw.leak[2] = false;
  hw.leak[3] = false;
#if ENABLE_MPU6050
  if (mpu6050Ok) {
    readMpu6050(&hw.pitch, &hw.roll, &hw.yaw);
  } else {
    hw.pitch = 0.0f;
    hw.roll = 0.0f;
    hw.yaw = 0.0f;
  }
#else
  hw.pitch = 0.0f;
  hw.roll = 0.0f;
  hw.yaw = 0.0f;
#endif
  hw.fault = hw.leak[0] ? "LEAK" : "NONE";

  for (int i = 0; i < BALLAST_COUNT; i++) {
    updateBallastAdc(&ballasts[i]);
  }
}

static void printPins() {
  LINK.println("OK PINS");
  LINK.print("  I2C SDA="); LINK.print(I2C_SDA);
  LINK.print(" SCL="); LINK.println(I2C_SCL);
  LINK.print("  PCA9685 ch0=aftY ch1=aftZ ch2=finL ch3=finR ch4=sonar @0x");
  LINK.println(PCA9685_ADDR, HEX);
#if ENABLE_SONAR
  LINK.print("  Sonar TRIG="); LINK.print(PIN_SONAR_TRIG);
  LINK.print(" ECHO="); LINK.println(PIN_SONAR_ECHO);
#endif
#if ENABLE_MPU6050
  LINK.print("  MPU6050 GY-521 @0x");
  LINK.println(MPU6050_ADDR, HEX);
#endif
  LINK.print("  Thruster IN1="); LINK.print(PIN_THR_IN1);
  LINK.print(" IN2="); LINK.print(PIN_THR_IN2);
  LINK.print(" PWM="); LINK.println(PIN_THR_PWM);
  LINK.print("  Fore ballast DIR="); LINK.print(PIN_FORE_BALLAST_DIR);
  LINK.print(" PWM="); LINK.print(PIN_FORE_BALLAST_PWM);
  LINK.print(" pot="); LINK.println(PIN_FORE_BALLAST_POT);
  LINK.print("  Aft ballast DIR="); LINK.print(PIN_AFT_BALLAST_DIR);
  LINK.print(" PWM="); LINK.print(PIN_AFT_BALLAST_PWM);
  LINK.print(" pot="); LINK.println(PIN_AFT_BALLAST_POT);
  for (int i = 0; i < BALLAST_COUNT; i++) {
    BallastState *t = &ballasts[i];
    LINK.print("  Cal "); LINK.print(t->name);
    LINK.print(" bottom="); LINK.print(t->adcBottom);
    LINK.print(" top="); LINK.print(t->adcTop);
    LINK.print(" valid="); LINK.println(t->calValid ? 1 : 0);
  }
  LINK.print("  Battery ADC="); LINK.print(PIN_BATTERY_ADC);
  LINK.print(" Depth ADC="); LINK.println(PIN_DEPTH_ADC);
  LINK.print("  Leak GPIO ");
#if PIN_LEAK >= 0
  LINK.println(PIN_LEAK);
#else
  LINK.println("disabled");
#endif
#if USE_PI_UART
  LINK.print("  Pi UART RX="); LINK.print(PI_UART_RX);
  LINK.print(" TX="); LINK.println(PI_UART_TX);
#else
  LINK.println("  Pi link: USB Serial");
#endif
}

static void sendTelemetry() {
  readSensors();
  LINK.print("TEL battery ");
  LINK.println(hw.batteryV, 2);
  LINK.print("TEL gyro ");
  LINK.print(hw.pitch, 2); LINK.print(" ");
  LINK.print(hw.roll, 2); LINK.print(" ");
  LINK.println(hw.yaw, 2);
  LINK.print("TEL depth ");
  LINK.println(hw.depthM, 2);
  LINK.print("TEL leak ");
  LINK.print(hw.leak[0] ? 1 : 0); LINK.print(" ");
  LINK.print(hw.leak[1] ? 1 : 0); LINK.print(" ");
  LINK.print(hw.leak[2] ? 1 : 0); LINK.print(" ");
  LINK.println(hw.leak[3] ? 1 : 0);

  for (int i = 0; i < BALLAST_COUNT; i++) {
    BallastState *t = &ballasts[i];
    LINK.print("TEL ballast ");
    LINK.print(t->name); LINK.print(" ");
    // level: 0..1 raw ADC fraction, or calibrated 0..1 after CAL; -1 = pot GPIO unwired
    float levelOut = (t->pinPot >= 0 && t->adc >= 0) ? t->pos : -1.0f;
    LINK.print(levelOut, 3); LINK.print(" ");
    LINK.print(t->adc); LINK.print(" ");
    LINK.print(t->moving ? 1 : 0); LINK.print(" ");
    LINK.println(t->dir);
    LINK.print("TEL ballastcal ");
    LINK.print(t->name); LINK.print(" ");
    LINK.print(t->adcBottom); LINK.print(" ");
    LINK.print(t->adcTop); LINK.print(" ");
    LINK.println(t->calValid ? 1 : 0);
  }

  LINK.println("TEL controls");
  LINK.println(hw.aftY, 3);
  LINK.println(hw.aftZ, 3);
  LINK.println(hw.finL, 3);
  LINK.println(hw.finR, 3);
  LINK.println(hw.thruster, 3);
  LINK.println(ballasts[BALLAST_FORE].command, 3);
  LINK.println(ballasts[BALLAST_AFT].command, 3);
  LINK.print("TEL thruster ");
  LINK.print(hw.thruster, 3); LINK.print(" ");
  LINK.println(hw.thrusterPwm);
  LINK.print("TEL status ");
  LINK.println(hw.status);
  LINK.print("TEL fault ");
  LINK.println(hw.fault);
  LINK.print("TEL heartbeat ");
  LINK.println(hw.heartbeat++);
}

static void handleTestCommand(char *line) {
  char sub[16] = {};
  if (sscanf(line, "TEST %15s", sub) != 1) return;

  if (strcmp(sub, "S") == 0) {
    int ch = 0;
    float val = 0.0f;
    if (sscanf(line, "TEST S %d %f", &ch, &val) == 2) {
      testMode = true;
      testModeUntil = millis() + 3000;
      setServoChannel(ch, val);
      LINK.print("OK TEST servo ch="); LINK.print(ch);
      LINK.print(" val="); LINK.println(val, 3);
    }
    return;
  }
  if (strcmp(sub, "T") == 0) {
    float val = 0.0f;
    if (sscanf(line, "TEST T %f", &val) == 1) {
      testMode = true;
      testModeUntil = millis() + 3000;
      setThruster(val);
      LINK.print("OK TEST thruster val="); LINK.println(val, 3);
    }
    return;
  }
  if (strcmp(sub, "B") == 0) {
    char tankName[16] = {};
    char dir[16] = {};
    if (sscanf(line, "TEST B %15s %15s", tankName, dir) == 2) {
      int idx = ballastIndexFromName(tankName);
      if (idx < 0) return;
      testMode = true;
      testModeUntil = millis() + 3000;
      if (strcmp(dir, "fill") == 0) setBallastTank(&ballasts[idx], 1.0f);
      else if (strcmp(dir, "drain") == 0) setBallastTank(&ballasts[idx], -1.0f);
      else setBallastTank(&ballasts[idx], 0.0f);
      LINK.print("OK TEST ballast "); LINK.print(tankName);
      LINK.print(" "); LINK.println(dir);
    } else if (sscanf(line, "TEST B %15s", dir) == 1) {
      testMode = true;
      testModeUntil = millis() + 3000;
      float cmd = 0.0f;
      if (strcmp(dir, "fill") == 0) cmd = 1.0f;
      else if (strcmp(dir, "drain") == 0) cmd = -1.0f;
      setBallastBoth(cmd, cmd);
      LINK.print("OK TEST ballast both "); LINK.println(dir);
    }
    return;
  }
  if (strcmp(sub, "L") == 0) {
    readSensors();
    LINK.print("OK TEST leaks ");
    LINK.print(hw.leak[0]); LINK.print(" ");
    LINK.print(hw.leak[1]); LINK.print(" ");
    LINK.print(hw.leak[2]); LINK.print(" ");
    LINK.println(hw.leak[3]);
    return;
  }
  if (strcmp(sub, "A") == 0) {
    readSensors();
    LINK.print("OK TEST adc battery="); LINK.print(hw.batteryV, 2);
    LINK.print(" fore_gpio="); LINK.print(PIN_FORE_BALLAST_POT);
    LINK.print(" fore_adc="); LINK.print(ballasts[BALLAST_FORE].adc);
    LINK.print(" fore_v="); LINK.print(adcRawToVolts(ballasts[BALLAST_FORE].adc), 3);
    LINK.print(" aft_gpio="); LINK.print(PIN_AFT_BALLAST_POT);
    LINK.print(" aft_adc="); LINK.print(ballasts[BALLAST_AFT].adc);
    LINK.print(" aft_v="); LINK.print(adcRawToVolts(ballasts[BALLAST_AFT].adc), 3);
    LINK.print(" depth="); LINK.println(hw.depthM, 2);
    return;
  }
  if (strcmp(sub, "I") == 0) {
    int16_t ax, ay, az, gx, gy, gz;
    readSensors();
    LINK.print("OK TEST imu ok="); LINK.print(mpu6050Ok ? 1 : 0);
    LINK.print(" addr=0x"); LINK.print(mpu6050Addr, HEX);
    LINK.print(" pitch="); LINK.print(hw.pitch, 2);
    LINK.print(" roll="); LINK.print(hw.roll, 2);
    LINK.print(" yaw="); LINK.println(hw.yaw, 2);
    if (mpu6050Ok && mpu6050ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
      LINK.print("  raw ax="); LINK.print(ax);
      LINK.print(" ay="); LINK.print(ay);
      LINK.print(" az="); LINK.print(az);
      LINK.print(" gx="); LINK.print(gx);
      LINK.print(" gy="); LINK.print(gy);
      LINK.print(" gz="); LINK.println(gz);
    }
    return;
  }
  LINK.println("ERR TEST unknown subcommand");
}

static void handleCalCommand(char *line) {
  char tankName[16] = {};
  char which[16] = {};
  if (sscanf(line + 6, "%15s %15s", tankName, which) == 2) {
    int idx = ballastIndexFromName(tankName);
    if (idx < 0) {
      LINK.println("ERR CAL B use fore|aft top|bottom|show");
      return;
    }
    BallastState *t = &ballasts[idx];
    if (strcmp(which, "top") == 0) {
      calibrateBallastTop(t);
      return;
    }
    if (strcmp(which, "bottom") == 0) {
      calibrateBallastBottom(t);
      return;
    }
    if (strcmp(which, "show") == 0) {
      LINK.print("OK CAL B ");
      LINK.print(t->name);
      LINK.print(" show ");
      LINK.print(t->adcBottom); LINK.print(" ");
      LINK.print(t->adcTop); LINK.print(" ");
      LINK.println(t->calValid ? 1 : 0);
      return;
    }
  } else if (sscanf(line + 6, "%15s", which) == 1) {
    if (strcmp(which, "top") == 0) {
      calibrateBallastTop(&ballasts[BALLAST_FORE]);
      return;
    }
    if (strcmp(which, "bottom") == 0) {
      calibrateBallastBottom(&ballasts[BALLAST_FORE]);
      return;
    }
  }
  LINK.println("ERR CAL B use <fore|aft> top|bottom|show");
}

static void parseLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (line[0] == '\0') return;

  lastSerial = millis();
  testMode = false;

  if (strcmp(line, "PING") == 0) {
    sayLine("OK PONG");
    return;
  }
  if (strcmp(line, "PINS") == 0) {
    printPins();
    return;
  }
  if (strcmp(line, "HELP") == 0 || strcmp(line, "help") == 0) {
    LINK.println("OK HELP");
    LINK.println("  PING  PINS  HELP");
    LINK.println("  B <fore> <aft>  or  B fore <val>  B aft <val>");
    LINK.println("  CAL B <fore|aft> top|bottom|show");
    LINK.println("  S2 <aftY> <aftZ> F <finL> <finR> X <thruster>");
    LINK.println("  TEST B <fore|aft> fill|drain|stop  (or both)");
    LINK.println("  TEST S <ch> <val>  TEST T <val>  TEST L  TEST A  TEST I");
    return;
  }
  if (strncmp(line, "TEST ", 5) == 0) {
    handleTestCommand(line);
    return;
  }
  if (strncmp(line, "CAL B ", 6) == 0) {
    handleCalCommand(line);
    return;
  }

  float foreCmd = 0.0f;
  float aftCmd = 0.0f;
  char tankName[16] = {};
  float oneCmd = 0.0f;
  if (sscanf(line, "B %f %f", &foreCmd, &aftCmd) == 2) {
    setBallastBoth(foreCmd, aftCmd);
    return;
  }
  if (sscanf(line, "B %15s %f", tankName, &oneCmd) == 2) {
    int idx = ballastIndexFromName(tankName);
    if (idx == BALLAST_FORE) setBallastBoth(oneCmd, ballasts[BALLAST_AFT].command);
    else if (idx == BALLAST_AFT) setBallastBoth(ballasts[BALLAST_FORE].command, oneCmd);
    return;
  }
  if (sscanf(line, "B %f", &oneCmd) == 1) {
    setBallastBoth(oneCmd, oneCmd);
    return;
  }

  float aftY, aftZ, finL, finR, thr;
  if (sscanf(line, "S2 %f %f F %f %f X %f", &aftY, &aftZ, &finL, &finR, &thr) == 5) {
    setActuators(aftY, aftZ, finL, finR, thr);
    return;
  }

  float steer, drive, tilt;
  if (sscanf(line, "S %f D %f T %f", &steer, &drive, &tilt) == 3) {
    setActuators(steer, tilt, 0.0f, 0.0f, drive);
    return;
  }
}

static void processLinkStream() {
  while (LINK.available()) {
    char c = LINK.read();
    if (c == '\n' || c == '\r') {
      if (rxPos > 0) {
        rxBuf[rxPos] = '\0';
        parseLine(rxBuf);
        rxPos = 0;
      }
    } else if (rxPos < RX_BUF - 1) {
      rxBuf[rxPos++] = c;
    } else {
      rxPos = 0;
    }
  }
#if USB_DEBUG_MIRROR
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxPos > 0) {
        rxBuf[rxPos] = '\0';
        parseLine(rxBuf);
        rxPos = 0;
      }
    } else if (rxPos < RX_BUF - 1) {
      rxBuf[rxPos++] = c;
    } else {
      rxPos = 0;
    }
  }
#endif
}

static bool anyBallastActive() {
  for (int i = 0; i < BALLAST_COUNT; i++) {
    if (fabsf(ballasts[i].command) > BALLAST_CMD_DEADBAND) return true;
  }
  return false;
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1500);  // allow USB CDC to connect before first traffic
  bootCheckpoint("CHK0 serial up");

#if USE_PI_UART
  PiLink.begin(SERIAL_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
  piUartReady = true;
  bootCheckpoint("CHK1 pi uart 43/44");
#endif

  sayLine("sub_rc boot...");
#if USE_PI_UART
  sayLine("USE_PI_UART=1 PiLink RX=44 TX=43");
#else
  sayLine("USE_PI_UART=0 Pi link: USB Serial");
#endif

#if defined(ARDUINO_ARCH_ESP32)
  WiFi.mode(WIFI_OFF);
  WiFi.disconnect(true);
#endif

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

#if ENABLE_PCA9685 || ENABLE_MPU6050
  i2cBegin();
  bootCheckpoint("CHK2 i2c begin");
#endif

#if ENABLE_PCA9685
  if (pca9685.begin()) {
    i2cBegin();  // Adafruit library calls Wire.begin() without pins — restore SDA/SCL
    pca9685.setOscillatorFrequency(PCA9685_OSC_FREQ);
    pca9685.setPWMFreq(50);
    pca9685Ok = true;
    LINK.print("OK PCA9685 on I2C SDA="); LINK.print(I2C_SDA);
    LINK.print(" SCL="); LINK.println(I2C_SCL);
  } else {
    LINK.println("WARN PCA9685 not found — servos disabled");
  }
#else
  sayLine("PCA9685 disabled (ENABLE_PCA9685=0)");
#endif

#if ENABLE_MPU6050
  if (mpu6050Init()) {
    mpu6050Ok = true;
    LINK.print("OK MPU6050 GY-521 on I2C @0x");
    LINK.println(mpu6050Addr, HEX);
  } else {
    LINK.println("WARN MPU6050 not found — gyro telemetry zeros");
  }
#else
  sayLine("MPU6050 disabled (ENABLE_MPU6050=0)");
#endif

  pinMode(PIN_THR_IN1, OUTPUT);
  pinMode(PIN_THR_IN2, OUTPUT);
  pinMode(PIN_FORE_BALLAST_DIR, OUTPUT);
  pinMode(PIN_FORE_BALLAST_PWM, OUTPUT);
  pinMode(PIN_AFT_BALLAST_DIR, OUTPUT);
  pinMode(PIN_AFT_BALLAST_PWM, OUTPUT);
#if PIN_LEAK >= 0
  pinMode(PIN_LEAK, INPUT_PULLDOWN);  // floating pin reads LOW (no leak)
#endif
#if ENABLE_SONAR
  if (PIN_SONAR_TRIG >= 0) pinMode(PIN_SONAR_TRIG, OUTPUT);
  if (PIN_SONAR_ECHO >= 0) pinMode(PIN_SONAR_ECHO, INPUT);
#endif
  bootCheckpoint("CHK3 pin modes");

  digitalWrite(PIN_THR_IN1, LOW);
  digitalWrite(PIN_THR_IN2, LOW);
  digitalWrite(PIN_FORE_BALLAST_DIR, LOW);
  digitalWrite(PIN_FORE_BALLAST_PWM, LOW);
  digitalWrite(PIN_AFT_BALLAST_DIR, LOW);
  digitalWrite(PIN_AFT_BALLAST_PWM, LOW);

#if PIN_THR_PWM >= 0
  ledcAttach(PIN_THR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_THR_PWM, 0);
#endif
  bootCheckpoint("CHK4 thruster pwm attached");

  loadBallastCal();
  bootCheckpoint("CHK5 nvs loaded");

  setActuators(0, 0, 0, 0, 0);
  stopAllBallast();
#if ENABLE_SONAR
  if (pca9685Ok && PIN_SONAR_TRIG >= 0) {
    setSonarServoDeg(SONAR_SWEEP_MAX);
  }
#endif
  bootCheckpoint("CHK6 actuators idle");

  hw.status = "READY";
  hw.fault = "NONE";
  lastSerial = millis();
  lastTelem = millis();

  sayLine("sub_rc ready - send PING / PINS / HELP / B / S2");
  printPins();
}

void loop() {
  processLinkStream();

#if ENABLE_SONAR
  updateSonarSweep();
#endif

  if (testMode && millis() > testModeUntil) {
    testMode = false;
    setActuators(0, 0, 0, 0, 0);
    stopAllBallast();
    setThruster(0);
  }

  unsigned long now = millis();
  if (now - lastTelem >= (1000UL / TELEMETRY_HZ)) {
    lastTelem = now;
    sendTelemetry();
  }

  if (now - lastSerial > SERIAL_TIMEOUT_MS) {
    if (lastThruster != 0.0f || anyBallastActive()) {
      lastThruster = 0.0f;
      stopAllBallast();
      setThruster(0);
      hw.status = "SERIAL_TIMEOUT";
    }
  } else {
    hw.status = "READY";
    if (!testMode) {
      setThruster(lastThruster);
    }
  }
}
