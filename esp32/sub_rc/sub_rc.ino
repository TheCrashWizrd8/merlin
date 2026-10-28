// ESP32-S3 — Sub vehicle receiver + sensor hub
// Pi link: USB Serial (default) or GPIO UART — see USE_PI_UART below.
//
// PINOUT — edit these to match your wiring (-1 = not connected)

// GPIO UART (optional): cross Pi GPIO14 TX / GPIO15 RX with ESP RX/TX
#define USE_PI_UART           0
#define PI_UART_RX            44    // ESP RX <- Pi TX (header pin 8)
#define PI_UART_TX            43    // ESP TX -> Pi RX (header pin 10)

// I2C bus — shared by PCA9685 (servos) and MPU6050 (GY-521 IMU)
// Wired: SDA=GPIO8, SCL=GPIO9. Do not use GPIO 29/30 (flash).
#define I2C_SDA               8
#define I2C_SCL               9
#define PCA9685_ADDR          0x40    // all jumpers OPEN; closed A0=0x41 … A5=0x60
#define ENABLE_PCA9685        1
#define MPU6050_ADDR          0x68    // GY-521 default (AD0 → GND)
#define ENABLE_MPU6050        1

// PCA9685 channels — ESC on 0; sonar on 1; four fin servos on headers 2–5
#define CH_THRUSTER_ESC       0     // bidirectional ESC signal (50 Hz servo pulse)
#define CH_SONAR_SERVO        1
#define CH_FIN_2              2
#define CH_FIN_3              3
#define CH_FIN_4              4
#define CH_FIN_5              5
// Blue Robotics Basic ESC @ 50 Hz PCA (~4.883 µs/tick). Thruster usage guide:
// stop/init 1500 µs, reverse 1100 µs, forward 1900 µs; PWM must stay on (≤400 Hz).
// Fin ticks 150..600 are ~730..2930 µs — invalid for Basic ESC (no second init beep).
#define ESC_US_STOP           1500
#define ESC_US_REVERSE        1100
#define ESC_US_FORWARD        1900
// Must match pca9685Program50Hz prescale (121 → ~4.883 µs/tick, true 50 Hz).
#define PCA9685_PWM_PRESCALE_50HZ  121
// Continuous 1500 µs while commanded throttle is zero (Pi triggers released).
// RT/LT ignored until this completes (software safety after ESC init).
#define ESC_ARM_HOLD_MS       5000
#define ESC_PWM_REFRESH_MS    20    // ~50 Hz re-write (ESC accepts up to 400 Hz)
// S2 line: S2 <aftY> <aftZ> F <finL> <finR> X <thr>  →  ch3 ch2 ch4 ch5
#define CH_AFT_STEER_Y        CH_FIN_3
#define CH_AFT_STEER_Z        CH_FIN_2
#define CH_FIN_LEFT           CH_FIN_4
#define CH_FIN_RIGHT          CH_FIN_5

// Sonar — HC-SR04 / JSN-SR04T style (trigger + echo). Set ENABLE_SONAR=0 to disable.
#define ENABLE_SONAR          1
#define PIN_SONAR_TRIG        17
#define PIN_SONAR_ECHO        18
#define SONAR_MAX_RANGE_M     6.0f
#define SONAR_STEP_DEG        5
#define SONAR_SWEEP_MIN       -180
#define SONAR_SWEEP_MAX       180
#define SONAR_SETTLE_MS       80
#define SONAR_PULSE_TIMEOUT_US 12000

// Legacy L298N pins. Thruster is the ESC on PCA channel 0. Set -1 when unwired.
// GPIO 6 is fore ballast pot (ADC1) — do not use as L298N PWM.
#define PIN_THR_IN1           13
#define PIN_THR_IN2           12
#define PIN_THR_PWM           -1

// Ballast — Makerverse Motor Driver 2 Channel (DIR/PWM mode, on/off fill/drain)
// Fore: DIR=16 PWM=15. Aft: DIR=4 PWM=5.
#define BALLAST_USE_DIR_PWM   1
#define PIN_FORE_BALLAST_DIR  16    // DIR A (IO16)
#define PIN_FORE_BALLAST_PWM  15    // PWM A (IO15)
// Pot wipers: ESP32-S3 ADC1 (GPIO 1–10). Avoid strapping pins 0, 3, 45, 46.
// Set BALLAST_POT_GPIO_SWAP 1 if aft slide pot is on GPIO 6 and fore on GPIO 2.
#define BALLAST_POT_GPIO_SWAP 0
#if BALLAST_POT_GPIO_SWAP
#define PIN_FORE_BALLAST_POT  2
#define PIN_AFT_BALLAST_POT   6
#else
#define PIN_FORE_BALLAST_POT  6     // ADC1_CH5 — fore pot wiper (HUD tank 1)
#define PIN_AFT_BALLAST_POT   2     // ADC1_CH1 — aft pot wiper (HUD tank 2)
#endif
#define PIN_AFT_BALLAST_DIR   4     // DIR B (IO4)
#define PIN_AFT_BALLAST_PWM   5     // PWM B (IO5)

// Sensors
#define PIN_BATTERY_ADC       10    // ADC1 (was GPIO 2 — now aft ballast pot)
#define PIN_DEPTH_ADC         7     // ADC1
#define DEPTH_I2C_ADDR        -1    // TODO: I2C depth sensor address when wired

// Leak board — 4-zone combined signal on IO1 / D1 (active HIGH).
#define PIN_LEAK              1     // set -1 when unwired
#define LEAK_DEBOUNCE_HITS    3
#define LEAK_DEBOUNCE_SAMPLES 5
#define LEAK_ZONE_COUNT       4     // zones on the board (one combined GPIO)

// Timing
#define SERIAL_BAUD           115200
#define SERIAL_TIMEOUT_MS     8000
#define TELEMETRY_HZ          5
#define MOTOR_PWM_FREQ        5000
#define MOTOR_PWM_RES         8
#define MOTOR_MAX_SPEED       255
#define MOTOR_DEADBAND        0.08f   // ignore small S2 / trigger noise before ESC moves
#define MOTOR_MIN_START       90
#define BALLAST_CMD_DEADBAND  0.05f
#define BALLAST_CAL_STOP_MARGIN 0.02f  // hold the outward command within 2% of cal top/bottom
#define BALLAST_DIR_LEARN_COUNTS 40    // ADC change used to learn which command raises the pot
#define SERVO_REST_DEADBAND     0.02f  // hold calibrated centre; ignore tick-hunt near 0

// Adafruit PCA9685 pulse limits (same as working bench sketch: setPWM(ch,0,150..600))
#define SERVO_MIN             150
#define SERVO_MAX             600
// Neutral command (0): 96.5° on 0..180° → linear tick map 150..600
#define SERVO_CENTER_DEG      96.5f
#define SERVO_CENTER_TICK     391

static uint16_t servoCenterTickCh[16] = {};
static uint16_t servoMinTickCh[16] = {};
static uint16_t servoMaxTickCh[16] = {};
#define PCA9685_LED0_ON_L     0x06

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
#if defined(ARDUINO_ARCH_ESP32)
#include <WiFi.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>
#endif

#if USE_PI_UART
HardwareSerial PiLink(1);
#define LINK PiLink
#else
#define LINK Serial
#endif

#if ENABLE_PCA9685
static uint8_t pca9685Addr = PCA9685_ADDR;
#endif

struct BallastState {
  int pinIna;   // Makerverse DIR
  int pinInb;   // Makerverse PWM (enable)
  int pinPot;
  float command;
  float pos;
  int adc;
  int mvMilli;   // linear mV from ADC counts (DMM on ESP GPIO pin)
  bool moving;
  const char *dir;
  int adcTop;
  int adcBottom;
  bool calValid;
  const char *name;
  const char *nvsTopKey;
  const char *nvsBotKey;
  const char *nvsCalKey;
  int motionAdc;       // previous ADC sample used to learn pump direction
  float motionCmd;     // command that was active during that sample
  int8_t fillRaisesPos; // 1 fill increases cal pos, -1 decreases, 0 not yet seen
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
    0.0f, 0.0f, -1, -1, false, "STOP", -1, -1, false, "fore", "fTop", "fBot", "fCal", -1, 0.0f, 0 },
  { PIN_AFT_BALLAST_DIR, PIN_AFT_BALLAST_PWM, PIN_AFT_BALLAST_POT,
    0.0f, 0.0f, -1, -1, false, "STOP", -1, -1, false, "aft", "aTop", "aBot", "aCal", -1, 0.0f, 0 },
};

unsigned long lastSerial = 0;
unsigned long lastTelem = 0;
float lastThruster = 0.0f;
float pendingAftY = 0.0f, pendingAftZ = 0.0f, pendingFinL = 0.0f, pendingFinR = 0.0f, pendingThr = 0.0f;
bool actuatorsPending = false;
bool testMode = false;
unsigned long testModeUntil = 0;
bool servoCalActive = false;
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

#if ENABLE_PCA9685
static bool pcaWritePWM(uint8_t ch, uint16_t on, uint16_t off);
#endif

static uint16_t defaultCenterTick() {
  return (uint16_t)SERVO_CENTER_TICK;
}

static uint16_t centerTickForChannel(int ch) {
  if (ch < 0 || ch > 15) return defaultCenterTick();
  uint16_t t = servoCenterTickCh[ch];
  if (t < SERVO_MIN || t > SERVO_MAX) return defaultCenterTick();
  return t;
}

static uint16_t minTickForChannel(int ch) {
  if (ch < 0 || ch > 15) return (uint16_t)SERVO_MIN;
  uint16_t t = servoMinTickCh[ch];
  if (t < SERVO_MIN || t > SERVO_MAX) return (uint16_t)SERVO_MIN;
  uint16_t center = centerTickForChannel(ch);
  if (t > center) t = center;
  return t;
}

static uint16_t maxTickForChannel(int ch) {
  if (ch < 0 || ch > 15) return (uint16_t)SERVO_MAX;
  uint16_t t = servoMaxTickCh[ch];
  if (t < SERVO_MIN || t > SERVO_MAX) return (uint16_t)SERVO_MAX;
  uint16_t center = centerTickForChannel(ch);
  if (t < center) t = center;
  return t;
}

static uint16_t servoTickForChannel(int ch, float v) {
  v = clampf(v);
  const uint16_t center = centerTickForChannel(ch);
  const uint16_t tMin = minTickForChannel(ch);
  const uint16_t tMax = maxTickForChannel(ch);
  float pulse;
  if (v >= 0.0f) {
    pulse = (float)center + v * (float)(tMax - center);
  } else {
    pulse = (float)center + v * (float)(center - tMin);
  }
  if (pulse < (float)tMin) pulse = (float)tMin;
  if (pulse > (float)tMax) pulse = (float)tMax;
  return (uint16_t)(pulse + 0.5f);
}

static void saveServoCenterChannel(int ch, uint16_t tick, bool persist) {
  if (ch < 0 || ch > 15) return;
  if (tick < SERVO_MIN) tick = SERVO_MIN;
  if (tick > SERVO_MAX) tick = SERVO_MAX;
  servoCenterTickCh[ch] = tick;
  if (persist) {
    char key[8];
    snprintf(key, sizeof(key), "sC%d", ch);
    ballastPrefs.begin("sub_rc", false);
    ballastPrefs.putInt(key, (int)tick);
    ballastPrefs.end();
  }
}

static void saveServoTravelChannel(int ch, bool isMax, uint16_t tick, bool persist) {
  if (ch < 0 || ch > 15) return;
  if (tick < SERVO_MIN) tick = SERVO_MIN;
  if (tick > SERVO_MAX) tick = SERVO_MAX;
  uint16_t center = centerTickForChannel(ch);
  if (isMax) {
    if (tick <= center) tick = (center < SERVO_MAX) ? (uint16_t)(center + 1) : (uint16_t)SERVO_MAX;
    servoMaxTickCh[ch] = tick;
  } else {
    if (tick >= center) tick = (center > SERVO_MIN) ? (uint16_t)(center - 1) : (uint16_t)SERVO_MIN;
    servoMinTickCh[ch] = tick;
  }
  if (persist) {
    char key[8];
    snprintf(key, sizeof(key), isMax ? "sMx%d" : "sMn%d", ch);
    ballastPrefs.begin("sub_rc", false);
    ballastPrefs.putInt(key, (int)tick);
    ballastPrefs.end();
  }
}

static void loadServoCentersFromNvs() {
  for (int i = 0; i < 16; i++) {
    servoCenterTickCh[i] = defaultCenterTick();
    servoMinTickCh[i] = (uint16_t)SERVO_MIN;
    servoMaxTickCh[i] = (uint16_t)SERVO_MAX;
  }
  ballastPrefs.begin("sub_rc", true);
  for (int ch = CH_FIN_2; ch <= CH_FIN_5; ch++) {
    char key[8];
    snprintf(key, sizeof(key), "sC%d", ch);
    int t = ballastPrefs.getInt(key, -1);
    if (t >= (int)SERVO_MIN && t <= (int)SERVO_MAX) {
      servoCenterTickCh[ch] = (uint16_t)t;
    }
    uint16_t center = centerTickForChannel(ch);
    snprintf(key, sizeof(key), "sMn%d", ch);
    t = ballastPrefs.getInt(key, -1);
    if (t >= (int)SERVO_MIN && t < (int)center) {
      servoMinTickCh[ch] = (uint16_t)t;
    }
    snprintf(key, sizeof(key), "sMx%d", ch);
    t = ballastPrefs.getInt(key, -1);
    if (t > (int)center && t <= (int)SERVO_MAX) {
      servoMaxTickCh[ch] = (uint16_t)t;
    }
  }
  ballastPrefs.end();
}

static void beginServoCalHold() {
  servoCalActive = true;
  testMode = true;
  testModeUntil = millis() + 300000;
}

static void endServoCalHold() {
  servoCalActive = false;
  testMode = false;
}

static void handleCalServoCommand(char *line) {
  int ch = -1;
  int tick = 0;
  if (sscanf(line + 6, "TUNE MIN %d %d", &ch, &tick) == 2 ||
      sscanf(line + 6, "TUNE MAX %d %d", &ch, &tick) == 2) {
    bool isMax = (line[12] == 'A');
    beginServoCalHold();
    if (!pca9685Ok || ch < 0 || ch > 15) {
      LINK.println("ERR CAL S TUNE pca=0 or bad ch");
      return;
    }
    if (tick < (int)SERVO_MIN) tick = SERVO_MIN;
    if (tick > (int)SERVO_MAX) tick = SERVO_MAX;
    pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
    LINK.print("OK CAL S TUNE "); LINK.print(isMax ? "MAX" : "MIN");
    LINK.print(" ch="); LINK.print(ch);
    LINK.print(" tick="); LINK.println(tick);
    return;
  }
  if (sscanf(line + 6, "MIN %d %d", &ch, &tick) == 2) {
    saveServoTravelChannel(ch, false, (uint16_t)tick, true);
    if (pca9685Ok && ch >= 0 && ch <= 15) {
      pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
    }
    LINK.print("OK CAL S MIN "); LINK.print(ch);
    LINK.print(" "); LINK.println(tick);
    return;
  }
  if (sscanf(line + 6, "MAX %d %d", &ch, &tick) == 2) {
    saveServoTravelChannel(ch, true, (uint16_t)tick, true);
    if (pca9685Ok && ch >= 0 && ch <= 15) {
      pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
    }
    LINK.print("OK CAL S MAX "); LINK.print(ch);
    LINK.print(" "); LINK.println(tick);
    return;
  }
  if (sscanf(line + 6, "TUNE %d %d", &ch, &tick) == 2 ||
      sscanf(line + 6, "T %d %d", &ch, &tick) == 2) {
    beginServoCalHold();
    if (!pca9685Ok || ch < 0 || ch > 15) {
      LINK.println("ERR CAL S TUNE pca=0 or bad ch");
      return;
    }
    if (tick < (int)SERVO_MIN) tick = SERVO_MIN;
    if (tick > (int)SERVO_MAX) tick = SERVO_MAX;
    pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
    LINK.print("OK CAL S TUNE ch="); LINK.print(ch);
    LINK.print(" tick="); LINK.println(tick);
    return;
  }
  if (sscanf(line + 6, "%d %d", &ch, &tick) == 2) {
    saveServoCenterChannel(ch, (uint16_t)tick, true);
    if (pca9685Ok && ch >= 0 && ch <= 15) {
      pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
    }
    LINK.print("OK CAL S "); LINK.print(ch);
    LINK.print(" "); LINK.println(tick);
    return;
  }
  if (strcmp(line + 6, "DONE") == 0) {
    endServoCalHold();
    LINK.println("OK CAL S DONE");
    return;
  }
  if (strcmp(line + 6, "SHOW") == 0) {
    LINK.println("OK CAL S SHOW");
    for (int c = CH_FIN_2; c <= CH_FIN_5; c++) {
      LINK.print("  ch"); LINK.print(c);
      LINK.print(" center="); LINK.print(centerTickForChannel(c));
      LINK.print(" min="); LINK.print(minTickForChannel(c));
      LINK.print(" max="); LINK.println(maxTickForChannel(c));
    }
    return;
  }
  LINK.println("ERR CAL S use TUNE <ch> <tick> | TUNE MIN|MAX <ch> <tick> | MIN|MAX <ch> <tick> | SHOW");
}

#if ENABLE_PCA9685
static uint16_t lastServoOff[16] = {};
static bool lastServoOk[16] = {};

// Same 5-byte write as Adafruit_PWMServoDriver::setPWM(ch, 0, pulse).
// Bench: map(angle, 0, 180, SERVO_MIN, SERVO_MAX). Runtime: servoTick(-1..1) with centre at 96.5°.
static bool pcaWritePWM(uint8_t ch, uint16_t on, uint16_t off) {
  if (ch > 15) return false;
  Wire.beginTransmission(pca9685Addr);
  Wire.write((uint8_t)(PCA9685_LED0_ON_L + 4 * ch));
  Wire.write(on & 0xFF);
  Wire.write(on >> 8);
  Wire.write(off & 0xFF);
  Wire.write(off >> 8);
  if (Wire.endTransmission() != 0) {
    lastServoOk[ch] = false;
    return false;
  }
  lastServoOff[ch] = off;
  lastServoOk[ch] = true;
  return true;
}
#endif

static void setServoChannel(int ch, float v) {
#if ENABLE_PCA9685
  if (!pca9685Ok || ch < 0 || ch > 15) return;
  uint16_t tick = servoTickForChannel(ch, v);
  // Rewriting an unchanged PCA9685 pulse glitches the servo (buzz at rest).
  if (lastServoOk[ch] && lastServoOff[ch] == tick) return;
  pcaWritePWM((uint8_t)ch, 0, tick);
#else
  (void)ch;
  (void)v;
#endif
}

static void setFinServos(float v2, float v3, float v4, float v5) {
  setServoChannel(CH_FIN_2, v2);
  setServoChannel(CH_FIN_3, v3);
  setServoChannel(CH_FIN_4, v4);
  setServoChannel(CH_FIN_5, v5);
}

// The sweep steps ch1 through ±180° ~10×/s whenever the fins are idle, which looks
// like servo jitter. Opt-in: the Pi sends SONAR 1 only if sonar.sweep_enabled is set.
static bool sonarSweepEnabled = false;

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
  if (!sonarSweepEnabled) return;
  if (!pca9685Ok || PIN_SONAR_TRIG < 0 || PIN_SONAR_ECHO < 0) return;
  if (testMode) return;
  // Let fin S2 writes win on a busy shared I2C bus.
  if (fabsf(hw.aftY) > 0.02f || fabsf(hw.aftZ) > 0.02f ||
      fabsf(hw.finL) > 0.02f || fabsf(hw.finR) > 0.02f) {
    return;
  }

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

static uint16_t lastEscTick = 0xFFFF;
static unsigned long lastEscWriteMs = 0;
static unsigned long escNeutralSinceMs = 0;
static bool escThrottleAllowed = false;

// The PCA9685 internal oscillator is nominally 25 MHz but varies per board. On this board
// tick 307 (25 MHz) missed the ESC's 1500 µs stop window and tick 320 (26 MHz) missed it too;
// tick 314 (25.5 MHz) gave the final stop beep, so ~25.5 MHz is the measured value.
// Retune live with: ESC OSC <MHz>.
#define ESC_OSC_MHZ_DEFAULT   25.5f
static float escOscMhz = ESC_OSC_MHZ_DEFAULT;

static uint16_t escUsToTick(int us) {
  if (us < 0) us = 0;
  return (uint16_t)(((float)us * escOscMhz) / (float)(PCA9685_PWM_PRESCALE_50HZ + 1) + 0.5f);
}

static unsigned escTickToUs(uint16_t tick) {
  return (unsigned)(((float)tick * (float)(PCA9685_PWM_PRESCALE_50HZ + 1)) / escOscMhz + 0.5f);
}

static uint16_t escTickForCommand(float v) {
  int us = ESC_US_STOP;
  if (v > 0.0f) {
    us = ESC_US_STOP + (int)(v * (float)(ESC_US_FORWARD - ESC_US_STOP) + 0.5f);
  } else if (v < 0.0f) {
    us = ESC_US_STOP + (int)(v * (float)(ESC_US_STOP - ESC_US_REVERSE) - 0.5f);
  }
  if (us < ESC_US_REVERSE) us = ESC_US_REVERSE;
  if (us > ESC_US_FORWARD) us = ESC_US_FORWARD;
  return escUsToTick(us);
}

#if ENABLE_PCA9685
static void escWritePwmTick(uint16_t tick, bool force) {
  if (!pca9685Ok) return;
  unsigned long nowMs = millis();
  if (!force && tick == lastEscTick && lastServoOk[CH_THRUSTER_ESC] &&
      (nowMs - lastEscWriteMs) < ESC_PWM_REFRESH_MS) {
    return;
  }
  if (pcaWritePWM((uint8_t)CH_THRUSTER_ESC, 0, tick)) {
    lastEscTick = tick;
    lastEscWriteMs = nowMs;
  }
}

static void escWriteNeutral(bool force) {
  escWritePwmTick(escUsToTick(ESC_US_STOP), force);
}

// Keep PCA ch0 pulsing even when fin I2C is busy (Basic ESC needs continuous PWM).
static void escMaintainOutput() {
  if (!pca9685Ok) return;
  unsigned long nowMs = millis();
  if (nowMs - lastEscWriteMs < ESC_PWM_REFRESH_MS) return;
  float req = lastThruster;
  if (fabsf(req) <= MOTOR_DEADBAND) req = 0.0f;
  if (!escThrottleAllowed || req == 0.0f) {
    escWriteNeutral(true);
    return;
  }
  escWritePwmTick(escTickForCommand(req), true);
}

// Stream 1500 µs stop while RT/LT command is zero. Throttle is blocked until this
// runs continuously for ESC_ARM_HOLD_MS (works when the ESC is powered after boot).
static void escUpdateNeutralHold(float req) {
  unsigned long nowMs = millis();
  if (fabsf(req) > MOTOR_DEADBAND) {
    if (!escThrottleAllowed) {
      escNeutralSinceMs = 0;
      escWriteNeutral(false);
    }
    return;
  }
  escWriteNeutral(false);
  if (escNeutralSinceMs == 0) {
    escNeutralSinceMs = nowMs;
    if (!escThrottleAllowed) {
      sayLine("OK ESC arming (1500us stop, release RT/LT)");
    }
  }
  if (!escThrottleAllowed && nowMs - escNeutralSinceMs >= ESC_ARM_HOLD_MS) {
    escThrottleAllowed = true;
    sayLine("OK ESC ready (Basic ESC init beeps should be done)");
  }
}
#endif

static void setThruster(float requested) {
  float req = clampf(requested);
  if (fabsf(req) <= MOTOR_DEADBAND) req = 0.0f;
  lastThruster = req;

  float v = req;
#if ENABLE_PCA9685
  escUpdateNeutralHold(req);
  if (!escThrottleAllowed) {
    v = 0.0f;
  }
#endif

  hw.thruster = v;
  hw.thrusterPwm = (v == 0.0f) ? 0 : (int)(fabsf(v) * MOTOR_MAX_SPEED);

#if ENABLE_PCA9685
  if (!pca9685Ok) return;
  if (!escThrottleAllowed || v == 0.0f) {
    escWriteNeutral(false);
    return;
  }
  escWritePwmTick(escTickForCommand(v), false);
#endif
}

static float adcRawToVolts(int adc) {
  if (adc < 0) return -1.0f;
  return (adc / 4095.0f) * 3.3f;
}

// ESP32-S3 ADC1 is GPIO 1–10 (not the classic ESP32 map of GPIO 32–39).
// GPIO2 = ADC1_CH1, GPIO6 = ADC1_CH5. 12-bit, ADC_11db ≈ 0–3.3 V.
// Do not pinMode(INPUT) these pads: that enables the digital input buffer and
// leaves the RTC pull-up on. The SAR then sits at 4095 until a meter loads the pin.
static bool s3Adc1Gpio(int pin) {
  return pin >= 1 && pin <= 10;
}

static void floatS3AdcPad(int pin) {
#if defined(ARDUINO_ARCH_ESP32)
  gpio_num_t g = (gpio_num_t)pin;
  gpio_pullup_dis(g);
  gpio_pulldown_dis(g);
  if (rtc_gpio_is_valid_gpio(g)) {
    rtc_gpio_pullup_dis(g);
    rtc_gpio_pulldown_dis(g);
    rtc_gpio_hold_dis(g);
  }
#else
  (void)pin;
#endif
}

static bool initPotAdcPin(int pin) {
  if (!s3Adc1Gpio(pin)) return false;
#if defined(ARDUINO_ARCH_ESP32)
  gpio_config_t conf = {};
  conf.pin_bit_mask = 1ULL << pin;
  conf.mode = GPIO_MODE_DISABLE;  // digital in/out off — analog pad only
  conf.pull_up_en = GPIO_PULLUP_DISABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&conf);
  if (rtc_gpio_is_valid_gpio((gpio_num_t)pin)) {
    rtc_gpio_init((gpio_num_t)pin);
  }
  floatS3AdcPad(pin);
#endif
  (void)analogRead((uint8_t)pin);  // binds oneshot ADC1 channel (Arduino 3.3 has no adcAttachPin)
  analogSetPinAttenuation((uint8_t)pin, ADC_11db);
  floatS3AdcPad(pin);
  (void)analogRead((uint8_t)pin);
  return true;
}

static int lastBallastAdcPin = -1;

static uint32_t medianMv(const uint32_t *buf, int n) {
  uint32_t tmp[11];
  if (n > 11) n = 11;
  for (int i = 0; i < n; i++) tmp[i] = buf[i];
  for (int i = 1; i < n; i++) {
    uint32_t v = tmp[i];
    int j = i;
    while (j > 0 && tmp[j - 1] > v) {
      tmp[j] = tmp[j - 1];
      j--;
    }
    tmp[j] = v;
  }
  return tmp[n / 2];
}

static int readBallastAdc(int pin, int *mvOut) {
  if (pin < 0) return -1;
  // Attenuation is set once in initPotAdcPin(). Reconfiguring it on every sample
  // pegs the ESP32-S3 oneshot ADC at full scale (~3.3 V).
  if (pin != lastBallastAdcPin) {
    for (int k = 0; k < 4; k++) {
      (void)analogRead((uint8_t)pin);
      delayMicroseconds(500);
    }
    lastBallastAdcPin = pin;
  }
  const int samples = 9;
  uint32_t rawBuf[samples];
  for (int i = 0; i < samples; i++) {
    rawBuf[i] = (uint32_t)analogRead((uint8_t)pin);
    delayMicroseconds(300);
  }
  int rawMed = (int)medianMv(rawBuf, samples);
  if (rawMed < 0) rawMed = 0;
  if (rawMed > 4095) rawMed = 4095;
  uint32_t mv = (uint32_t)(((uint64_t)rawMed * 3300UL + 2047UL) / 4095UL);
  if (mvOut) *mvOut = (int)mv;
  return rawMed;
}

static void noteBallastDirection(BallastState *tank);
static void applyBallastOutput(BallastState *tank);

static void updateBallastAdc(BallastState *t) {
  if (t->pinPot < 0) {
    t->adc = -1;
    t->mvMilli = -1;
    t->pos = -1.0f;
    return;
  }
  int mv = -1;
  int adc = readBallastAdc(t->pinPot, &mv);
  t->adc = adc;
  t->mvMilli = mv;
  t->pos = ballastPosFromAdc(t, t->adc);
  noteBallastDirection(t);
  applyBallastOutput(t);
}

static float ballastPosFromAdc(BallastState *tank, int adc) {
  if (!tank->calValid || tank->adcTop < 0 || tank->adcBottom < 0) {
    return adc / 4095.0f;
  }
  float span = (float)(tank->adcTop - tank->adcBottom);
  if (fabsf(span) < 50.0f) {
    return adc / 4095.0f;
  }
  // Signed span: top/bottom order follows physical wiring (ADC may rise or fall when filling).
  return clampf((adc - tank->adcBottom) / span);
}

static void loadBallastCal() {
  ballastPrefs.begin("sub_rc", true);
  for (int i = 0; i < BALLAST_COUNT; i++) {
    BallastState *t = &ballasts[i];
    t->adcTop = ballastPrefs.getInt(t->nvsTopKey, -1);
    t->adcBottom = ballastPrefs.getInt(t->nvsBotKey, -1);
    t->calValid = ballastPrefs.getBool(t->nvsCalKey, false);
    if (!t->calValid || t->adcTop < 0 || t->adcBottom < 0 ||
        abs(t->adcTop - t->adcBottom) < 50) {
      t->calValid = false;
      t->adcTop = -1;
      t->adcBottom = -1;
    }
  }
  ballastPrefs.end();
}

static void saveBallastCal(BallastState *tank) {
  ballastPrefs.begin("sub_rc", false);
  if (tank->calValid && tank->adcTop >= 0 && tank->adcBottom >= 0) {
    ballastPrefs.putInt(tank->nvsTopKey, tank->adcTop);
    ballastPrefs.putInt(tank->nvsBotKey, tank->adcBottom);
    ballastPrefs.putBool(tank->nvsCalKey, true);
  } else {
    // Remove keys so a reboot cannot reload old stop points.
    ballastPrefs.remove(tank->nvsTopKey);
    ballastPrefs.remove(tank->nvsBotKey);
    ballastPrefs.remove(tank->nvsCalKey);
  }
  ballastPrefs.end();
}

static void finalizeBallastCal(BallastState *tank) {
  if (tank->adcTop >= 0 && tank->adcBottom >= 0) {
    int span = abs(tank->adcTop - tank->adcBottom);
    if (span >= 50) {
      tank->calValid = true;
    } else {
      tank->calValid = false;
      LINK.print("WARN CAL B ");
      LINK.print(tank->name);
      LINK.print(" span=");
      LINK.print(span);
      LINK.println(" (need >= 50 ADC counts between top and bottom)");
    }
  }
  tank->fillRaisesPos = 0;
  tank->motionAdc = -1;
  tank->motionCmd = 0.0f;
  saveBallastCal(tank);
  updateBallastAdc(tank);
}

static void sendBallastCalTel(BallastState *t) {
  LINK.print("TEL ballastcal ");
  LINK.print(t->name);
  LINK.print(" ");
  if (t->calValid && t->adcTop >= 0 && t->adcBottom >= 0) {
    LINK.print(t->adcBottom);
    LINK.print(" ");
    LINK.print(t->adcTop);
    LINK.print(" ");
    LINK.println(1);
  } else {
    LINK.println("-1 -1 0");
  }
}

static void clearBallastCal(BallastState *tank) {
  tank->adcTop = -1;
  tank->adcBottom = -1;
  tank->calValid = false;
  tank->fillRaisesPos = 0;
  tank->motionAdc = -1;
  tank->motionCmd = 0.0f;
  saveBallastCal(tank);
  updateBallastAdc(tank);
  sendBallastCalTel(tank);
  LINK.print("OK CAL B ");
  LINK.print(tank->name);
  LINK.println(" clear");
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
  tank->adcTop = readBallastAdc(tank->pinPot, nullptr);
  finalizeBallastCal(tank);
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
  tank->adcBottom = readBallastAdc(tank->pinPot, nullptr);
  finalizeBallastCal(tank);
  LINK.print("OK CAL B ");
  LINK.print(tank->name);
  LINK.print(" bottom ");
  LINK.println(tank->adcBottom);
}

static void ballastPwmEnable(int pin, bool on) {
  if (pin < 0) return;
  ledcWrite(pin, on ? MOTOR_MAX_SPEED : 0);
}

// Raw calibrated position. 0 is the saved bottom, 1 is the saved top.
// Values outside 0..1 mean the pot is past that stop.
static float ballastPosRaw(BallastState *tank, int adc) {
  if (!tank->calValid || tank->adcTop < 0 || tank->adcBottom < 0 || adc < 0) {
    return adc >= 0 ? adc / 4095.0f : -1.0f;
  }
  float span = (float)(tank->adcTop - tank->adcBottom);
  if (fabsf(span) < 50.0f) return adc / 4095.0f;
  return (adc - (float)tank->adcBottom) / span;
}

// Watch the pot while a command is running and remember which sign moves
// toward the calibrated top. The end stop uses that so the return direction
// still runs when the pot is past a stop.
static void noteBallastDirection(BallastState *tank) {
  int prev = tank->motionAdc;
  float prevCmd = tank->motionCmd;
  bool cmdChanged = fabsf(tank->command - prevCmd) > 0.01f;
  tank->motionAdc = tank->adc;
  tank->motionCmd = tank->command;
  if (cmdChanged || prev < 0 || tank->adc < 0 || !tank->calValid) return;
  if (fabsf(prevCmd) <= BALLAST_CMD_DEADBAND) return;
  int dAdc = tank->adc - prev;
  if (abs(dAdc) < BALLAST_DIR_LEARN_COUNTS) return;
  float span = (float)(tank->adcTop - tank->adcBottom);
  if (fabsf(span) < 50.0f) return;
  bool posUp = ((float)dAdc / span) > 0.0f;
  bool cmdUp = prevCmd > 0.0f;
  tank->fillRaisesPos = (posUp == cmdUp) ? 1 : -1;
}

// True only for the command that travels further past this end.
// The opposite command is left alone.
static bool ballastCommandBlocked(BallastState *tank, float cmd) {
  if (!tank->calValid || tank->fillRaisesPos == 0 || tank->adc < 0) return false;
  bool fill = cmd > BALLAST_CMD_DEADBAND;
  bool drain = cmd < -BALLAST_CMD_DEADBAND;
  if (!fill && !drain) return false;
  float pos = ballastPosRaw(tank, tank->adc);
  bool fillRaises = tank->fillRaisesPos > 0;
  if (pos >= 1.0f - BALLAST_CAL_STOP_MARGIN) {
    return fillRaises ? fill : drain;
  }
  if (pos <= BALLAST_CAL_STOP_MARGIN) {
    return fillRaises ? drain : fill;
  }
  return false;
}

static void applyBallastOutput(BallastState *tank) {
  float cmd = tank->command;
  bool fill = cmd > BALLAST_CMD_DEADBAND;
  bool drain = cmd < -BALLAST_CMD_DEADBAND;
  if (ballastCommandBlocked(tank, cmd)) {
    fill = false;
    drain = false;
  }

#if BALLAST_USE_DIR_PWM
  if (fill) {
    digitalWrite(tank->pinIna, HIGH);
    ballastPwmEnable(tank->pinInb, true);
  } else if (drain) {
    digitalWrite(tank->pinIna, LOW);
    ballastPwmEnable(tank->pinInb, true);
  } else {
    digitalWrite(tank->pinIna, LOW);
    ballastPwmEnable(tank->pinInb, false);
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

static void ackBallast() {
  LINK.print("OK B ");
  LINK.print(ballasts[BALLAST_FORE].command, 3);
  LINK.print(" ");
  LINK.println(ballasts[BALLAST_AFT].command, 3);
}

static void setBallastTank(BallastState *tank, float cmd) {
  // Keep the requested command. A stop only drops the outward drive;
  // the opposite sign still reaches the pump.
  tank->command = clampf(cmd);
  updateBallastAdc(tank);
}

static void setBallastBoth(float foreCmd, float aftCmd) {
  setBallastTank(&ballasts[BALLAST_FORE], foreCmd);
  setBallastTank(&ballasts[BALLAST_AFT], aftCmd);
}

static void stopAllBallast() {
  setBallastBoth(0.0f, 0.0f);
}

static float snapServoRest(float v) {
  v = clampf(v);
  if (fabsf(v) < SERVO_REST_DEADBAND) return 0.0f;
  return v;
}

static void setActuators(float aftY, float aftZ, float finL, float finR, float thr) {
  float y = snapServoRest(aftY);
  float z = snapServoRest(aftZ);
  float fl = snapServoRest(finL);
  float fr = snapServoRest(finR);
  bool changed = (y != hw.aftY) || (z != hw.aftZ) || (fl != hw.finL) || (fr != hw.finR);
  hw.aftY = y;
  hw.aftZ = z;
  hw.finL = fl;
  hw.finR = fr;
#if ENABLE_PCA9685
  uint16_t tk2 = servoTickForChannel(CH_FIN_2, hw.aftZ);
  uint16_t tk3 = servoTickForChannel(CH_FIN_3, hw.aftY);
  uint16_t tk4 = servoTickForChannel(CH_FIN_4, hw.finL);
  uint16_t tk5 = servoTickForChannel(CH_FIN_5, hw.finR);
  bool pulseChanged = pca9685Ok &&
      (tk2 != lastServoOff[CH_FIN_2] || tk3 != lastServoOff[CH_FIN_3] ||
       tk4 != lastServoOff[CH_FIN_4] || tk5 != lastServoOff[CH_FIN_5]);
  bool needRetry = pca9685Ok &&
      (!lastServoOk[CH_FIN_2] || !lastServoOk[CH_FIN_3] ||
       !lastServoOk[CH_FIN_4] || !lastServoOk[CH_FIN_5]);
#else
  bool pulseChanged = false;
  bool needRetry = false;
#endif
  if (changed || needRetry || pulseChanged) {
    setFinServos(hw.aftZ, hw.aftY, hw.finL, hw.finR);
  }
  setThruster(thr);
  if (!pca9685Ok) {
    static unsigned long lastPcaErrMs = 0;
    unsigned long now = millis();
    if (now - lastPcaErrMs > 3000) {
      lastPcaErrMs = now;
      LINK.println("ERR S2 pca=0 (PCA9685 not on I2C — check SDA=8 SCL=9, 3.3V, servo 5V)");
    }
  } else if (changed || pulseChanged) {
    LINK.print("OK S2 ");
    LINK.print(hw.aftY, 3); LINK.print(" ");
    LINK.print(hw.aftZ, 3); LINK.print(" F ");
    LINK.print(hw.finL, 3); LINK.print(" ");
    LINK.print(hw.finR, 3); LINK.print(" X ");
    LINK.print(hw.thruster, 3);
    LINK.print(" pca="); LINK.print(pca9685Ok ? 1 : 0);
#if ENABLE_PCA9685
    LINK.print(" tk2-5=");
    LINK.print(tk2); LINK.print(",");
    LINK.print(tk3); LINK.print(",");
    LINK.print(tk4); LINK.print(",");
    LINK.println(tk5);
#else
    LINK.println();
#endif
  }
}

static void queueActuators(float aftY, float aftZ, float finL, float finR, float thr) {
  pendingAftY = aftY;
  pendingAftZ = aftZ;
  pendingFinL = finL;
  pendingFinR = finR;
  pendingThr = thr;
  actuatorsPending = true;
}

static void applyPendingActuators() {
  if (!actuatorsPending) return;
  actuatorsPending = false;
  setActuators(pendingAftY, pendingAftZ, pendingFinL, pendingFinR, pendingThr);
}

static float readAdcVolts(int pin) {
  if (pin < 0) return 0.0f;
  int mv = -1;
  int raw = readBallastAdc(pin, &mv);
  if (raw < 0) return 0.0f;
  if (mv >= 0) return mv / 1000.0f;
  return (raw / 4095.0f) * 3.3f;
}

static bool readLeakActive() {
#if PIN_LEAK < 0
  return false;
#else
  int highs = 0;
  for (int i = 0; i < LEAK_DEBOUNCE_SAMPLES; i++) {
    if (digitalRead(PIN_LEAK) == HIGH) highs++;
    delayMicroseconds(400);
  }
  return highs >= LEAK_DEBOUNCE_HITS;
#endif
}

#if ENABLE_PCA9685 || ENABLE_MPU6050
static bool i2cStarted = false;

static void i2cBegin() {
  if (i2cStarted) return;
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(20);
  i2cStarted = true;
}

static void i2cRebegin() {
  if (i2cStarted) {
    Wire.end();
    i2cStarted = false;
    delay(5);
  }
  i2cBegin();
}
#endif

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
static uint8_t mpuFailCount = 0;
#endif

#if ENABLE_PCA9685
#define PCA9685_MODE1         0x00
#define PCA9685_PRESCALE      0xFE
#define PCA9685_SLEEP         0x10
#define PCA9685_AUTOINCR      0x20
#define PCA9685_RESTART       0x80
// 25 MHz / (4096 × 50 Hz) − 1 ≈ 121 (matches config/pins.yaml tick_us 4.8828).
// Prescale 135 was ~45 Hz and made tick 307 ≈ 1670 µs — Basic ESC beeps once, not at 1500 µs.
#define PCA9685_PRESCALE_50HZ PCA9685_PWM_PRESCALE_50HZ

static bool pcaWrite8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(pca9685Addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool pcaRead8(uint8_t addr, uint8_t reg, uint8_t *out) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
  *out = Wire.read();
  return true;
}

static bool pcaProbeAddr(uint8_t addr) {
  uint8_t mode1 = 0;
  return pcaRead8(addr, PCA9685_MODE1, &mode1);
}

static void i2cScanBus() {
  LINK.println("I2C scan:");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) continue;
    found++;
    LINK.print("  0x");
    if (addr < 16) LINK.print("0");
    LINK.print(addr, HEX);
    if (addr >= 0x40 && addr <= 0x7F) LINK.print("  PCA9685?");
    if (addr == 0x68 || addr == 0x69) LINK.print("  MPU6050");
    LINK.println();
  }
  if (!found) LINK.println("  none — check SDA=8 SCL=9, 3.3V, GND");
}

static bool pca9685Program50Hz() {
  // Match Adafruit_PWMServoDriver::setPWMFreq(50): sleep → prescale → wake → restart.
  for (int attempt = 0; attempt < 3; attempt++) {
    if (attempt > 0) delay(10);
    if (!pcaWrite8(PCA9685_MODE1, PCA9685_SLEEP | PCA9685_AUTOINCR)) continue;
    delay(5);
    if (!pcaWrite8(PCA9685_PRESCALE, PCA9685_PRESCALE_50HZ)) continue;
    if (!pcaWrite8(PCA9685_MODE1, PCA9685_AUTOINCR)) continue;
    delay(5);
    if (!pcaWrite8(PCA9685_MODE1, PCA9685_AUTOINCR | PCA9685_RESTART)) continue;
    delay(5);
    return true;
  }
  return false;
}

static bool pca9685Init() {
  i2cBegin();
  const uint32_t savedClock = 100000;
  Wire.setClock(50000);

  pca9685Addr = PCA9685_ADDR;
  bool found = false;
  for (int attempt = 0; attempt < 5 && !found; attempt++) {
    if (attempt > 0) delay(50);
    for (uint8_t addr = 0x40; addr <= 0x47; addr++) {
      if (!pcaProbeAddr(addr)) continue;
      pca9685Addr = addr;
      found = true;
      break;
    }
  }
  if (!found) {
    Wire.setClock(savedClock);
    pca9685Ok = false;
    LINK.println("WARN PCA9685 not found @0x40-0x47 (I2C scan follows)");
    i2cScanBus();
    return false;
  }

  if (!pca9685Program50Hz()) {
    Wire.setClock(savedClock);
    pca9685Ok = false;
    LINK.print("WARN PCA9685 @0x");
    LINK.print(pca9685Addr, HEX);
    LINK.println(" freq program failed — check 3.3V on PCA VCC");
    return false;
  }

  Wire.setClock(savedClock);
  pca9685Ok = true;
  escThrottleAllowed = false;
  escNeutralSinceMs = 0;
  LINK.println("Servo controller started");
  LINK.print("OK PCA9685 @0x");
  LINK.print(pca9685Addr, HEX);
  LINK.print(" SDA="); LINK.print(I2C_SDA);
  LINK.print(" SCL="); LINK.println(I2C_SCL);
  lastEscTick = 0xFFFF;
  escWriteNeutral(true);
  {
    uint16_t stopTick = escUsToTick(ESC_US_STOP);
    LINK.print("OK ESC BlueRobotics stop tick=");
    LINK.print(stopTick);
    LINK.print(" ~");
    LINK.print(escTickToUs(stopTick));
    LINK.print("us prescale=");
    LINK.println(PCA9685_PWM_PRESCALE_50HZ);
  }
  for (int ch = CH_FIN_2; ch <= CH_FIN_5; ch++) {
    pcaWritePWM((uint8_t)ch, 0, centerTickForChannel(ch));
  }
  if (ENABLE_SONAR) {
    pcaWritePWM((uint8_t)CH_SONAR_SERVO, 0, centerTickForChannel(CH_SONAR_SERVO));
  }
  return true;
}
#endif

#if ENABLE_MPU6050
static bool mpu6050WriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  if (!i2cStarted) i2cBegin();
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool mpu6050ReadByte(uint8_t addr, uint8_t reg, uint8_t *val) {
  if (!i2cStarted) i2cBegin();
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission() != 0) return false;  // STOP then read — avoids i2cWriteReadNonStop
  if (Wire.requestFrom((int)addr, 1) != 1) return false;
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
  if (!i2cStarted) i2cBegin();
  Wire.beginTransmission(mpu6050Addr);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  if (Wire.endTransmission() != 0) return false;
  if (Wire.requestFrom((int)mpu6050Addr, 14) != 14) return false;

  *ax = (int16_t)((Wire.read() << 8) | Wire.read());
  *ay = (int16_t)((Wire.read() << 8) | Wire.read());
  *az = (int16_t)((Wire.read() << 8) | Wire.read());
  Wire.read(); Wire.read();  // skip temperature
  *gx = (int16_t)((Wire.read() << 8) | Wire.read());
  *gy = (int16_t)((Wire.read() << 8) | Wire.read());
  *gz = (int16_t)((Wire.read() << 8) | Wire.read());
  while (Wire.available()) Wire.read();
  return true;
}

static void readMpu6050(float *pitchDeg, float *rollDeg, float *yawDeg) {
  int16_t ax, ay, az, gx, gy, gz;
  if (!mpu6050ReadRaw(&ax, &ay, &az, &gx, &gy, &gz)) {
    if (++mpuFailCount >= 5) {
      mpu6050Ok = false;
      i2cBegin();
    }
    *pitchDeg = 0.0f;
    *rollDeg = 0.0f;
    *yawDeg = 0.0f;
    return;
  }
  mpuFailCount = 0;

  float axG = ax / MPU6050_ACCEL_SCALE;
  float ayG = ay / MPU6050_ACCEL_SCALE;
  float azG = az / MPU6050_ACCEL_SCALE;
  // Chip-frame Euler. Vehicle-frame remap is config/hardware.yaml imu.mount
  // (GY-521 on its side reports ~90° roll until the Pi remaps gravity onto Z).
  *pitchDeg = atan2f(ayG, sqrtf(axG * axG + azG * azG)) * 57.2957795f;
  *rollDeg = atan2f(-axG, azG) * 57.2957795f;
  // No magnetometer — yaw is not meaningful; report 0 for dashboard compatibility.
  (void)gx; (void)gy; (void)gz;
  *yawDeg = 0.0f;
}
#endif

static void readSensors() {
  // Ballast pots first (GPIO 2 is noise-sensitive after other ADC1 channels).
  for (int i = 0; i < BALLAST_COUNT; i++) {
    updateBallastAdc(&ballasts[i]);
  }
  hw.depthM = readAdcVolts(PIN_DEPTH_ADC);
  hw.batteryV = readAdcVolts(PIN_BATTERY_ADC) * 4.0f;
  bool leakActive = readLeakActive();
  for (int i = 0; i < LEAK_ZONE_COUNT; i++) {
    hw.leak[i] = leakActive;
  }
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
  hw.fault = leakActive ? "LEAK" : "NONE";
}

static void printPins() {
  LINK.println("OK PINS");
  LINK.print("  I2C SDA="); LINK.print(I2C_SDA);
  LINK.print(" SCL="); LINK.println(I2C_SCL);
  LINK.print("  PCA9685 ch0=ESC ch1=sonar ch2-5=fins (S2 z y / F l r) @0x");
#if ENABLE_PCA9685
  LINK.print(pca9685Ok ? pca9685Addr : PCA9685_ADDR, HEX);
  LINK.println(pca9685Ok ? " ok" : " missing");
#else
  LINK.println(PCA9685_ADDR, HEX);
#endif
#if ENABLE_SONAR
  LINK.print("  Sonar TRIG="); LINK.print(PIN_SONAR_TRIG);
  LINK.print(" ECHO="); LINK.println(PIN_SONAR_ECHO);
#endif
#if ENABLE_MPU6050
  LINK.print("  MPU6050 GY-521 @0x");
  LINK.println(MPU6050_ADDR, HEX);
#endif
  LINK.print("  Thruster ESC PCA ch"); LINK.println(CH_THRUSTER_ESC);
  LINK.println("  neutral=stop  +1=forward (RT)  -1=reverse (LT)");
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
  LINK.print("  Leak board GPIO ");
#if PIN_LEAK >= 0
  LINK.print(PIN_LEAK);
  LINK.print(" (");
  LINK.print(LEAK_ZONE_COUNT);
  LINK.println(" zones, combined signal)");
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
    sendBallastCalTel(t);
    LINK.print("TEL ballast ");
    LINK.print(t->name); LINK.print(" ");
    // level: 0..1 raw ADC fraction, or calibrated 0..1 after CAL; -1 = pot GPIO unwired
    float levelOut = (t->pinPot >= 0 && t->adc >= 0) ? t->pos : -1.0f;
    LINK.print(levelOut, 3); LINK.print(" ");
    LINK.print(t->adc); LINK.print(" ");
    LINK.print(t->mvMilli); LINK.print(" ");
    LINK.print(t->moving ? 1 : 0); LINK.print(" ");
    LINK.println(t->dir);
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
#if ENABLE_PCA9685
  LINK.print("TEL esc ");
  LINK.print(escThrottleAllowed ? 1 : 0);
  LINK.print(" ");
  if (escNeutralSinceMs == 0) {
    LINK.print(0);
  } else {
    LINK.print((unsigned long)(millis() - escNeutralSinceMs));
  }
  LINK.print(" ");
  LINK.print(ESC_ARM_HOLD_MS);
  LINK.print(" ");
  if (lastEscTick == 0xFFFF) {
    LINK.println(0);
  } else {
    LINK.println(escTickToUs(lastEscTick));
  }
  LINK.print("TEL pca9685 ");
  LINK.print(pca9685Ok ? 1 : 0); LINK.print(" ");
  LINK.print(pca9685Ok ? (int)pca9685Addr : (int)PCA9685_ADDR, HEX); LINK.print(" ");
  LINK.print(I2C_SDA); LINK.print(" ");
  LINK.println(I2C_SCL);
#endif
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
      LINK.print(" val="); LINK.print(val, 3);
      LINK.print(" ticks="); LINK.print(servoTickForChannel(ch, val));
      LINK.print(" pca="); LINK.println(pca9685Ok ? 1 : 0);
    }
    return;
  }
  if (strcmp(sub, "P") == 0) {
    int ch = 0;
    int tick = 0;
    if (sscanf(line, "TEST P %d %d", &ch, &tick) == 2) {
      beginServoCalHold();
      if (!pca9685Ok || ch < 0 || ch > 15) {
        LINK.println("ERR TEST P pca=0 or bad ch");
        return;
      }
      if (tick < (int)SERVO_MIN) tick = SERVO_MIN;
      if (tick > (int)SERVO_MAX) tick = SERVO_MAX;
      pcaWritePWM((uint8_t)ch, 0, (uint16_t)tick);
      LINK.print("OK TEST P ch="); LINK.print(ch);
      LINK.print(" tick="); LINK.println(tick);
    }
    return;
  }
  if (strcmp(sub, "SWEEP") == 0) {
    if (!pca9685Ok) {
      LINK.println("ERR TEST SWEEP pca=0");
      return;
    }
    testMode = true;
    testModeUntil = millis() + 8000;
    for (int angle = 0; angle <= 180; angle++) {
      uint16_t pulse = (uint16_t)map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
      for (int ch = CH_FIN_2; ch <= CH_FIN_5; ch++) {
        pcaWritePWM((uint8_t)ch, 0, pulse);
      }
      delay(10);
    }
    for (int ch = CH_FIN_2; ch <= CH_FIN_5; ch++) {
      pcaWritePWM((uint8_t)ch, 0, centerTickForChannel(ch));
    }
    LINK.println("OK TEST SWEEP ch2-5");
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
    LINK.print(" fore_mv="); LINK.print(ballasts[BALLAST_FORE].mvMilli);
    LINK.print(" aft_gpio="); LINK.print(PIN_AFT_BALLAST_POT);
    LINK.print(" aft_adc="); LINK.print(ballasts[BALLAST_AFT].adc);
    LINK.print(" aft_mv="); LINK.print(ballasts[BALLAST_AFT].mvMilli);
    LINK.print(" depth="); LINK.println(hw.depthM, 2);
    LINK.println("OK TEST adc_scan (DMM on ESP GPIO pin; skip DIR/PWM 4,5)");
    for (int gpio = 1; gpio <= 10; gpio++) {
      if (gpio == 8 || gpio == 9 || gpio == 4 || gpio == 5) continue;
      int mv = -1;
      int raw = readBallastAdc(gpio, &mv);
      LINK.print("  GPIO");
      LINK.print(gpio);
      LINK.print(" adc=");
      LINK.print(raw);
      LINK.print(" mv=");
      LINK.println(mv);
    }
    return;
  }
  if (strcmp(sub, "P") == 0) {
    i2cRebegin();
    i2cScanBus();
#if ENABLE_PCA9685
    if (pca9685Init()) {
      LINK.println("OK TEST PCA reinit");
    } else {
      LINK.println("ERR TEST PCA reinit");
    }
#else
    LINK.println("ERR TEST PCA disabled");
#endif
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
    if ((strcmp(tankName, "both") == 0 || strcmp(tankName, "all") == 0) &&
        (strcmp(which, "clear") == 0 || strcmp(which, "reset") == 0)) {
      clearBallastCal(&ballasts[BALLAST_FORE]);
      clearBallastCal(&ballasts[BALLAST_AFT]);
      return;
    }
    int idx = ballastIndexFromName(tankName);
    if (idx < 0) {
      LINK.println("ERR CAL B use fore|aft|both top|bottom|show|clear");
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
    if (strcmp(which, "clear") == 0 || strcmp(which, "reset") == 0) {
      clearBallastCal(t);
      return;
    }
  } else if (sscanf(line + 6, "%15s", which) == 1) {
    if (strcmp(which, "clear") == 0 || strcmp(which, "reset") == 0) {
      clearBallastCal(&ballasts[BALLAST_FORE]);
      clearBallastCal(&ballasts[BALLAST_AFT]);
      return;
    }
    if (strcmp(which, "top") == 0) {
      calibrateBallastTop(&ballasts[BALLAST_FORE]);
      return;
    }
    if (strcmp(which, "bottom") == 0) {
      calibrateBallastBottom(&ballasts[BALLAST_FORE]);
      return;
    }
  }
  LINK.println("ERR CAL B use <fore|aft|both> top|bottom|show|clear");
}

static void parseLine(char *line) {
  while (*line == ' ' || *line == '\t') line++;
  if (line[0] == '\0') return;

  lastSerial = millis();

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
    LINK.println("  CAL S TUNE <ch> <tick>  |  TUNE MIN|MAX <ch> <tick>  |  MIN|MAX <ch> <tick>  |  SHOW");
    LINK.println("  CAL B <fore|aft|both> top|bottom|show|clear");
    LINK.println("  S2 <aftY> <aftZ> F <finL> <finR> X <thruster>");
    LINK.println("  ESC OSC <MHz>  (PCA9685 clock for ESC ticks, default 25.5; drops the signal 1.5 s so the ESC replays its beeps)");
    LINK.println("  SONAR <0|1>  (sonar servo sweep, default 0)");
    LINK.println("  TEST B <fore|aft> fill|drain|stop  (or both)");
    LINK.println("  TEST S <ch> <val>  TEST SWEEP  TEST P  TEST T <val>  TEST L  TEST A  TEST I");
    return;
  }
  if (strncmp(line, "TEST ", 5) == 0) {
    handleTestCommand(line);
    return;
  }
  float oscMhz = 0.0f;
  if (sscanf(line, "ESC OSC %f", &oscMhz) == 1) {
    if (oscMhz < 20.0f || oscMhz > 32.0f) {
      LINK.println("ERR ESC OSC range 20..32 MHz");
      return;
    }
#if ENABLE_PCA9685
    if (pca9685Ok) {
      // Drop the signal so the ESC re-runs its startup beeps with the new pulse width.
      pcaWritePWM((uint8_t)CH_THRUSTER_ESC, 0, 0x1000);  // LEDn_OFF_H bit 4 = full off
      delay(1500);
    }
#endif
    escOscMhz = oscMhz;
    lastEscTick = 0xFFFF;
    setThruster(lastThruster);  // re-write ch0 with the new tick scale
    LINK.print("OK ESC OSC "); LINK.print(escOscMhz, 2);
    LINK.print(" stop tick="); LINK.print(escUsToTick(ESC_US_STOP));
    LINK.print(" ~"); LINK.print(escTickToUs(escUsToTick(ESC_US_STOP)));
    LINK.println("us");
    return;
  }
  if (strcmp(line, "SONAR 0") == 0 || strcmp(line, "SONAR 1") == 0) {
    sonarSweepEnabled = (line[6] == '1');
#if ENABLE_SONAR
    if (sonarSweepEnabled) {
      sonarPhase = SONAR_MOVING;
    } else if (pca9685Ok && PIN_SONAR_TRIG >= 0) {
      setSonarServoDeg(0);
    }
#endif
    LINK.print("OK SONAR "); LINK.println(sonarSweepEnabled ? 1 : 0);
    return;
  }

  if (strncmp(line, "CAL S ", 6) == 0) {
    handleCalServoCommand(line);
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
    ackBallast();
    return;
  }
  if (sscanf(line, "B %15s %f", tankName, &oneCmd) == 2) {
    int idx = ballastIndexFromName(tankName);
    if (idx == BALLAST_FORE) setBallastBoth(oneCmd, ballasts[BALLAST_AFT].command);
    else if (idx == BALLAST_AFT) setBallastBoth(ballasts[BALLAST_FORE].command, oneCmd);
    ackBallast();
    return;
  }
  if (sscanf(line, "B %f", &oneCmd) == 1) {
    setBallastBoth(oneCmd, oneCmd);
    ackBallast();
    return;
  }

  float aftY, aftZ, finL, finR, thr;
  if (sscanf(line, "S2 %f %f F %f %f X %f", &aftY, &aftZ, &finL, &finR, &thr) == 5) {
    if (servoCalActive) return;
    if (fabsf(aftY) > 1.5f || fabsf(aftZ) > 1.5f || fabsf(finL) > 1.5f ||
        fabsf(finR) > 1.5f || fabsf(thr) > 1.5f) {
      LINK.println("ERR S2 range");
      return;
    }
    testMode = false;
    queueActuators(aftY, aftZ, finL, finR, thr);
    return;
  }

  float steer, drive, tilt;
  if (sscanf(line, "S %f D %f T %f", &steer, &drive, &tilt) == 3) {
    if (testMode) return;
    testMode = false;
    queueActuators(steer, tilt, 0.0f, 0.0f, drive);
    return;
  }

  if (testMode) {
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

  analogReadResolution(12);   // returned counts 0..4095 (S3 SAR width is 12-bit)
  analogSetAttenuation(ADC_11db);  // ~0–3.3 V full scale; default in this core is already 11 dB
  if (!initPotAdcPin(PIN_FORE_BALLAST_POT)) {
    sayLine("WARN fore ballast pot is not an ESP32-S3 ADC1 pin");
  }
  if (!initPotAdcPin(PIN_AFT_BALLAST_POT)) {
    sayLine("WARN aft ballast pot is not an ESP32-S3 ADC1 pin");
  }
  initPotAdcPin(PIN_BATTERY_ADC);
  initPotAdcPin(PIN_DEPTH_ADC);

  if (PIN_THR_IN1 >= 0) pinMode(PIN_THR_IN1, OUTPUT);
  if (PIN_THR_IN2 >= 0) pinMode(PIN_THR_IN2, OUTPUT);
  if (PIN_THR_PWM >= 0) pinMode(PIN_THR_PWM, OUTPUT);
  pinMode(PIN_FORE_BALLAST_DIR, OUTPUT);
  pinMode(PIN_FORE_BALLAST_PWM, OUTPUT);
  pinMode(PIN_AFT_BALLAST_DIR, OUTPUT);
  pinMode(PIN_AFT_BALLAST_PWM, OUTPUT);
#if PIN_LEAK >= 0
  pinMode(PIN_LEAK, INPUT_PULLDOWN);
#endif
#if ENABLE_SONAR
  if (PIN_SONAR_TRIG >= 0) pinMode(PIN_SONAR_TRIG, OUTPUT);
  if (PIN_SONAR_ECHO >= 0) pinMode(PIN_SONAR_ECHO, INPUT);
#endif
  bootCheckpoint("CHK2 pin modes");

  if (PIN_THR_IN1 >= 0) digitalWrite(PIN_THR_IN1, LOW);
  if (PIN_THR_IN2 >= 0) digitalWrite(PIN_THR_IN2, LOW);
  if (PIN_THR_PWM >= 0) digitalWrite(PIN_THR_PWM, LOW);
  digitalWrite(PIN_FORE_BALLAST_DIR, LOW);
  digitalWrite(PIN_AFT_BALLAST_DIR, LOW);

#if PIN_FORE_BALLAST_PWM >= 0
  ledcAttach(PIN_FORE_BALLAST_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ballastPwmEnable(PIN_FORE_BALLAST_PWM, false);
#endif
#if PIN_AFT_BALLAST_PWM >= 0
  ledcAttach(PIN_AFT_BALLAST_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ballastPwmEnable(PIN_AFT_BALLAST_PWM, false);
#endif
  // Other pinMode/ledcAttach calls must not leave the pot pads as digital inputs.
  floatS3AdcPad(PIN_FORE_BALLAST_POT);
  floatS3AdcPad(PIN_AFT_BALLAST_POT);
  floatS3AdcPad(PIN_BATTERY_ADC);
  floatS3AdcPad(PIN_DEPTH_ADC);

  bootCheckpoint("CHK3 pwm attached");

#if ENABLE_PCA9685 || ENABLE_MPU6050
  delay(100);  // let PCA9685 / IMU power settle after USB boot + GPIO setup
  i2cRebegin();
  bootCheckpoint("CHK4 i2c begin");
#endif

  loadServoCentersFromNvs();

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

#if ENABLE_PCA9685
  if (!pca9685Init()) {
    delay(200);
    i2cRebegin();
    pca9685Init();
  }
#else
  sayLine("PCA9685 disabled (ENABLE_PCA9685=0)");
#endif

  loadBallastCal();
  for (int i = 0; i < BALLAST_COUNT; i++) {
    updateBallastAdc(&ballasts[i]);
  }
  LINK.print("OK ballast pots fore_gpio=");
  LINK.print(PIN_FORE_BALLAST_POT);
  LINK.print(" adc=");
  LINK.print(ballasts[BALLAST_FORE].adc);
  LINK.print(" aft_gpio=");
  LINK.print(PIN_AFT_BALLAST_POT);
  LINK.print(" adc=");
  LINK.println(ballasts[BALLAST_AFT].adc);
  bootCheckpoint("CHK5 nvs loaded");

  setActuators(0, 0, 0, 0, 0);
  stopAllBallast();
#if ENABLE_SONAR
  if (pca9685Ok && PIN_SONAR_TRIG >= 0) {
    setSonarServoDeg(0);  // park at centre; the sweep only runs after SONAR 1
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
  if (!servoCalActive) {
    applyPendingActuators();
  }

#if ENABLE_SONAR
  updateSonarSweep();
#endif

  if (testMode && millis() > testModeUntil) {
    if (servoCalActive) {
      endServoCalHold();
    } else {
      testMode = false;
    }
    setActuators(0, 0, 0, 0, 0);
    stopAllBallast();
    setThruster(0);
  }

  unsigned long now = millis();

#if ENABLE_PCA9685
  if (!pca9685Ok) {
    static unsigned long lastPcaRetryMs = 0;
    if (now - lastPcaRetryMs >= 5000) {
      lastPcaRetryMs = now;
      i2cRebegin();
      pca9685Init();
    }
  } else {
    escMaintainOutput();
  }
#endif

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
  }
}
