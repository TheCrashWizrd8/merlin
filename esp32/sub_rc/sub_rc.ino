// ESP32-S3 — Sub vehicle receiver + sensor hub
// Pi link: USB Serial (default) or GPIO UART — see USE_PI_UART below.
//
// PINOUT — edit these to match your wiring (-1 = not connected)

// GPIO UART (optional): cross Pi GPIO14 TX / GPIO15 RX with ESP RX/TX
#define USE_PI_UART           0
#define PI_UART_RX            44    // ESP RX <- Pi TX (header pin 8)
#define PI_UART_TX            43    // ESP TX -> Pi RX (header pin 10)

// I2C / PCA9685 servo driver
// DO NOT use GPIO 26–32 or 29/30 on ESP32-S3 — those are internal flash/PSRAM (chip will
// watchdog-reset). Rewire PCA9685 SDA/SCL here:
#define I2C_SDA               21
#define I2C_SCL               22
#define PCA9685_ADDR          0x40
#define ENABLE_PCA9685        0     // 1 when PCA9685 wired on I2C_SDA/SCL (not GPIO 29/30!)

// PCA9685 channels: aft steer Y/Z, fore fins L/R
#define CH_AFT_STEER_Y        0
#define CH_AFT_STEER_Z        1
#define CH_FIN_LEFT           2
#define CH_FIN_RIGHT          3

// Thruster via L298N (IN2 moved — GPIO 5 is leak sensor)
#define PIN_THR_IN1           4
#define PIN_THR_IN2           12
#define PIN_THR_PWM           6

// Fore ballast — DC motor DIR + PWM + linear pot (-1 = pot not wired)
#define PIN_FORE_BALLAST_PWM  13
#define PIN_FORE_BALLAST_DIR  14
#define PIN_FORE_BALLAST_POT  7

// Aft ballast — DC motor DIR + PWM + linear pot (-1 = pot not wired)
#define PIN_AFT_BALLAST_DIR   8
#define PIN_AFT_BALLAST_PWM   9
#define PIN_AFT_BALLAST_POT   11

// Sensors
#define PIN_BATTERY_ADC       1     // ADC1
#define PIN_DEPTH_ADC         3     // ADC1 (or I2C depth sensor later)
#define DEPTH_I2C_ADDR        -1    // TODO: I2C depth sensor address when wired

// Leak detector (active HIGH = leak). Set to -1 when nothing is wired.
#define PIN_LEAK              -1    // was 5 — enable when sensor wired

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
#if ENABLE_PCA9685
#include <Wire.h>
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
  int pinPwm;
  int pinDir;
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
  { PIN_FORE_BALLAST_PWM, PIN_FORE_BALLAST_DIR, PIN_FORE_BALLAST_POT,
    0.0f, 0.0f, 0, false, "STOP", -1, -1, false, "fore", "fTop", "fBot", "fCal" },
  { PIN_AFT_BALLAST_PWM, PIN_AFT_BALLAST_DIR, PIN_AFT_BALLAST_POT,
    0.0f, 0.0f, 0, false, "STOP", -1, -1, false, "aft", "aTop", "aBot", "aCal" },
};

unsigned long lastSerial = 0;
unsigned long lastTelem = 0;
float lastThruster = 0.0f;
bool testMode = false;
unsigned long testModeUntil = 0;
bool pca9685Ok = false;
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
#if USB_DEBUG_MIRROR
  Serial.println(msg);
#endif
#if USE_PI_UART
  if (piUartReady) LINK.println(msg);
#else
  LINK.println(msg);
#endif
}

static void sayLine(const String &msg) {
#if USB_DEBUG_MIRROR
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
  if (!pca9685Ok || ch < 0 || ch > 3) return;
  pca9685.setPWM(ch, 0, servoTick(v));
#else
  (void)ch;
  (void)v;
#endif
}

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
  tank->adcTop = analogRead(tank->pinPot);
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
  tank->adcBottom = analogRead(tank->pinPot);
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
  float mag = fabsf(tank->command);
  int pwm = (int)(mag * MOTOR_MAX_SPEED);
  if (pwm > 0) pwm = max(MOTOR_MIN_START, pwm);
  pwm = min(MOTOR_MAX_SPEED, pwm);

  if (fill) {
    digitalWrite(tank->pinDir, HIGH);
    ledcWrite(tank->pinPwm, pwm);
  } else if (drain) {
    digitalWrite(tank->pinDir, LOW);
    ledcWrite(tank->pinPwm, pwm);
  } else {
    ledcWrite(tank->pinPwm, 0);
  }

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

static void readSensors() {
  hw.batteryV = readAdcVolts(PIN_BATTERY_ADC) * 4.0f;
  hw.depthM = readAdcVolts(PIN_DEPTH_ADC);
  hw.leak[0] = readLeakActive();
  hw.leak[1] = false;
  hw.leak[2] = false;
  hw.leak[3] = false;
  hw.pitch = 0.0f;
  hw.roll = 0.0f;
  hw.yaw = 0.0f;
  hw.fault = hw.leak[0] ? "LEAK" : "NONE";

  for (int i = 0; i < BALLAST_COUNT; i++) {
    BallastState *t = &ballasts[i];
    if (t->pinPot >= 0) {
      t->adc = analogRead(t->pinPot);
      t->pos = ballastPosFromAdc(t, t->adc);
    } else {
      t->adc = -1;
      t->pos = -1.0f;
    }
  }
}

static void printPins() {
  LINK.println("OK PINS");
  LINK.print("  I2C SDA="); LINK.print(I2C_SDA);
  LINK.print(" SCL="); LINK.println(I2C_SCL);
  LINK.print("  PCA9685 ch0=aftY ch1=aftZ ch2=finL ch3=finR @0x");
  LINK.println(PCA9685_ADDR, HEX);
  LINK.print("  Thruster IN1="); LINK.print(PIN_THR_IN1);
  LINK.print(" IN2="); LINK.print(PIN_THR_IN2);
  LINK.print(" PWM="); LINK.println(PIN_THR_PWM);
  LINK.print("  Fore ballast PWM="); LINK.print(PIN_FORE_BALLAST_PWM);
  LINK.print(" DIR="); LINK.print(PIN_FORE_BALLAST_DIR);
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
    // -1.000 level = pot unwired or uncalibrated (ignore on Pi/dashboard)
    LINK.print(t->calValid ? t->pos : -1.0f, 3); LINK.print(" ");
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
    LINK.print(" fore="); LINK.print(ballasts[BALLAST_FORE].adc);
    LINK.print(" aft="); LINK.print(ballasts[BALLAST_AFT].adc);
    LINK.print(" depth="); LINK.println(hw.depthM, 2);
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
    LINK.println("  TEST S <ch> <val>  TEST T <val>  TEST L  TEST A");
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
  sayLine("USE_PI_UART=1 PiLink RX=44 TX=43  PCA9685=off");
#else
  sayLine("USE_PI_UART=0 Pi link: USB Serial  PCA9685=off");
#endif

#if defined(ARDUINO_ARCH_ESP32)
  WiFi.mode(WIFI_OFF);
#endif

#if ENABLE_PCA9685
  Wire.begin(I2C_SDA, I2C_SCL);
  bootCheckpoint("CHK2 i2c begin");
  if (pca9685.begin()) {
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

  pinMode(PIN_THR_IN1, OUTPUT);
  pinMode(PIN_THR_IN2, OUTPUT);
  pinMode(PIN_FORE_BALLAST_DIR, OUTPUT);
  pinMode(PIN_AFT_BALLAST_DIR, OUTPUT);
#if PIN_LEAK >= 0
  pinMode(PIN_LEAK, INPUT_PULLDOWN);  // floating pin reads LOW (no leak)
#endif
  bootCheckpoint("CHK3 pin modes");

  digitalWrite(PIN_THR_IN1, LOW);
  digitalWrite(PIN_THR_IN2, LOW);
  digitalWrite(PIN_FORE_BALLAST_DIR, LOW);
  digitalWrite(PIN_AFT_BALLAST_DIR, LOW);

#if PIN_THR_PWM >= 0
  ledcAttach(PIN_THR_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_THR_PWM, 0);
#endif
  ledcAttach(PIN_FORE_BALLAST_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_FORE_BALLAST_PWM, 0);
  ledcAttach(PIN_AFT_BALLAST_PWM, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcWrite(PIN_AFT_BALLAST_PWM, 0);
  bootCheckpoint("CHK4 pwm attached");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  loadBallastCal();
  bootCheckpoint("CHK5 nvs loaded");

  setActuators(0, 0, 0, 0, 0);
  stopAllBallast();
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
