// ESP32-S3 - Sub vehicle receiver + sensor hub
// Pi link: USB (Serial) or GPIO UART — see USE_PI_UART below.
//
// PINOUT - set these for your wiring (-1 = not connected / disabled)

// Raspberry Pi UART (when USE_PI_UART=1): cross with Pi GPIO14 TX / GPIO15 RX
#define USE_PI_UART           1
#define PI_UART_RX            18    // ESP RX <- Pi GPIO14 (Pi TX, header pin 8)
#define PI_UART_TX            17    // ESP TX -> Pi GPIO15 (Pi RX, header pin 10)

// I2C bus (PCA9685 servo driver; optional IMU on same bus)
#define PIN_I2C_SDA           8
#define PIN_I2C_SCL           9
#define PCA9685_I2C_ADDR      0x40

// PCA9685 servo channels
#define CH_AFT_STEER_Y        0    // aft steering Y axis
#define CH_AFT_STEER_Z        1    // aft steering Z axis
#define CH_FIN_LEFT           2    // fore cap fin left
#define CH_FIN_RIGHT          3    // fore cap fin right

// Thruster (L298N or equivalent H-bridge)
#define PIN_THRUSTER_IN1      4
#define PIN_THRUSTER_IN2      5
#define PIN_THRUSTER_PWM      6

// Ballast (fill/drain outputs - pump, valve, or relay)
#define PIN_BALLAST_FILL      10   // active HIGH = fill
#define PIN_BALLAST_DRAIN     11   // active HIGH = drain
#define PIN_BALLAST_LEVEL_ADC  2   // ADC 0..3.3V -> 0..1; -1 = no level sensor

// Battery voltage (ADC divider on pack+)
#define PIN_BATTERY_ADC        1
#define BATTERY_ADC_VREF       3.3f
#define BATTERY_DIVIDER_RATIO  11.0f   // e.g. 10k+1k divider -> 11.0

// Leak sensors (digital, HIGH = leak detected); set unused pins to -1
#define PIN_LEAK_1            12
#define PIN_LEAK_2            13
#define PIN_LEAK_3            14
#define PIN_LEAK_4            15

// Depth sensor - Option A: analog pressure on ADC
#define PIN_DEPTH_ADC          3    // -1 = not used
#define DEPTH_ADC_MAX_M        10.0f // meters at full scale (calibrate!)
// Depth sensor - Option B: I2C address (MS58330 etc.); 0 = disabled
#define DEPTH_I2C_ADDR         0    // e.g. 0x76 when wired

// Gyro / IMU (MPU6050 on I2C); 0 = disabled, 0x69 if AD0 high
#define IMU_I2C_ADDR           0x68

// Serial protocol (see docs/SUB_DASHBOARD.md and src/esp_bridge.py)
// ESP -> Pi telemetry (every TELEMETRY_MS):
//   TEL battery <volts>
//   TEL gyro <pitch> <roll> <yaw>
//   TEL depth <meters>
//   TEL leak <0|1> [<0|1> ...]
//   TEL ballast <0.0..1.0>
// Pi -> ESP control (~20 Hz):
//   B <value>                                    (-1 drain, 0 hold, +1 fill)
//   S2 <aftY> <aftZ> F <finL> <finR> X <thr>    (all -1.0 .. +1.0)
// Legacy car line (optional): S <steer> D <drive> T <tilt>

#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>

#if USE_PI_UART
HardwareSerial PiLink(1);
#define LINK PiLink
#else
#define LINK Serial
#endif

// Timing and safety

#define SERIAL_BAUD           115200
#define SERIAL_TIMEOUT_MS     8000    // stop thruster if Pi goes silent
#define TELEMETRY_MS          200     // send TEL lines every 200 ms (5 Hz)
#define THRUSTER_PWM_FREQ     5000
#define THRUSTER_PWM_RES      8
#define THRUSTER_MAX_PWM      255
#define THRUSTER_DEADBAND     0.02f
#define THRUSTER_MIN_START    90

// PCA9685 servo pulse ticks @ 50 Hz (same as car sketch)
#define SERVO_MIN             205
#define SERVO_MAX             410
#define SERVO_CENTER          307
#define PCA9685_OSC_FREQ      25000000

// Leak pin table (edit PIN_LEAK_* above; unused slots = -1)

static const int LEAK_PINS[] = {
  PIN_LEAK_1, PIN_LEAK_2, PIN_LEAK_3, PIN_LEAK_4
};
static const int NUM_LEAKS = sizeof(LEAK_PINS) / sizeof(LEAK_PINS[0]);

// Globals

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDR);

unsigned long lastSerialMs = 0;
unsigned long lastTelemetryMs = 0;

float lastThruster = 0.0f;
float lastBallastCmd = 0.0f;
float reportedBallastLevel = 0.5f;  // updated from sensor or estimated

float telemBatteryV = 0.0f;
float telemPitch = 0.0f;
float telemRoll = 0.0f;
float telemYaw = 0.0f;
float telemDepthM = 0.0f;
bool  telemLeaks[4] = {false, false, false, false};
int   telemLeakCount = 0;

bool  pcaReady = false;
bool  imuReady = false;

#define RX_BUF 96
char rxBuf[RX_BUF];
int  rxPos = 0;

// Helpers

float clampf(float v)
{
  if (v < -1.0f) return -1.0f;
  if (v >  1.0f) return  1.0f;
  return v;
}

bool pinValid(int pin) { return pin >= 0; }

uint16_t servoTick(float v)
{
  v = clampf(v);
  float halfRange = (SERVO_MAX - SERVO_MIN) / 2.0f;
  return (uint16_t)(SERVO_CENTER + (v * halfRange));
}

void setServoChannel(uint8_t ch, float v)
{
  if (!pcaReady) return;
  pca9685.setPWM(ch, 0, servoTick(v));
}

float readAdcVolts(int pin)
{
  if (!pinValid(pin)) return 0.0f;
  int raw = analogRead(pin);
  return (raw / 4095.0f) * BATTERY_ADC_VREF;
}

// Thruster (L298N)

void setThruster(float v)
{
  if (!pinValid(PIN_THRUSTER_IN1) || !pinValid(PIN_THRUSTER_IN2) || !pinValid(PIN_THRUSTER_PWM))
    return;

  v = clampf(v);
  lastThruster = v;
  float mag = fabsf(v);
  int pwm = (int)(mag * THRUSTER_MAX_PWM);
  if (pwm > 0) pwm = max(THRUSTER_MIN_START, pwm);
  pwm = min(THRUSTER_MAX_PWM, pwm);

  if (v > THRUSTER_DEADBAND) {
    digitalWrite(PIN_THRUSTER_IN1, HIGH);
    digitalWrite(PIN_THRUSTER_IN2, LOW);
    ledcWrite(PIN_THRUSTER_PWM, pwm);
  } else if (v < -THRUSTER_DEADBAND) {
    digitalWrite(PIN_THRUSTER_IN1, LOW);
    digitalWrite(PIN_THRUSTER_IN2, HIGH);
    ledcWrite(PIN_THRUSTER_PWM, pwm);
  } else {
    digitalWrite(PIN_THRUSTER_IN1, LOW);
    digitalWrite(PIN_THRUSTER_IN2, LOW);
    ledcWrite(PIN_THRUSTER_PWM, 0);
  }
}

// Ballast outputs

void setBallast(float cmd)
{
  lastBallastCmd = clampf(cmd);
  bool fill = lastBallastCmd > 0.05f;
  bool drain = lastBallastCmd < -0.05f;

  if (pinValid(PIN_BALLAST_FILL))
    digitalWrite(PIN_BALLAST_FILL, fill ? HIGH : LOW);
  if (pinValid(PIN_BALLAST_DRAIN))
    digitalWrite(PIN_BALLAST_DRAIN, drain ? HIGH : LOW);

  // Simple level estimate when no ADC (integrate command - replace with real sensor)
  if (!pinValid(PIN_BALLAST_LEVEL_ADC)) {
    reportedBallastLevel = clampf(reportedBallastLevel + lastBallastCmd * 0.002f);
  }
}

// Sub actuator command: S2 y z F fl fr X thr

void parseSubActuators(char *line)
{
  float aftY, aftZ, finL, finR, thr;
  if (sscanf(line, "S2 %f %f F %f %f X %f", &aftY, &aftZ, &finL, &finR, &thr) != 5)
    return;

  lastSerialMs = millis();
  setServoChannel(CH_AFT_STEER_Y, aftY);
  setServoChannel(CH_AFT_STEER_Z, aftZ);
  setServoChannel(CH_FIN_LEFT, finL);
  setServoChannel(CH_FIN_RIGHT, finR);
  setThruster(thr);
}

void parseBallast(char *line)
{
  float b;
  if (sscanf(line, "B %f", &b) != 1)
    return;
  lastSerialMs = millis();
  setBallast(b);
}

// Legacy car line - kept in a separate block; does not touch sub channels.
void parseCarLine(char *line)
{
  float steer, drive, tilt;
  if (sscanf(line, "S %f D %f T %f", &steer, &drive, &tilt) != 3)
    return;
  lastSerialMs = millis();
  // Car mapping on shared PCA channels 0/1 if you still use S/D/T for bench tests
  setServoChannel(CH_AFT_STEER_Y, steer);
  setServoChannel(CH_AFT_STEER_Z, tilt);
  setThruster(drive);
}

// Diagnostic commands (bench / pin confirmation — see docs/pins_reference.md)
void printPinMap()
{
  LINK.println("OK PINS BEGIN");
  LINK.print("PIN pi_uart rx="); LINK.print(PI_UART_RX);
  LINK.print(" tx="); LINK.println(PI_UART_TX);
  LINK.print("PIN i2c sda="); LINK.print(PIN_I2C_SDA);
  LINK.print(" scl="); LINK.println(PIN_I2C_SCL);
  LINK.print("PIN pca9685 addr=0x"); LINK.println(PCA9685_I2C_ADDR, HEX);
  LINK.print("PIN servo ch_aft_y="); LINK.print(CH_AFT_STEER_Y);
  LINK.print(" ch_aft_z="); LINK.print(CH_AFT_STEER_Z);
  LINK.print(" ch_fin_l="); LINK.print(CH_FIN_LEFT);
  LINK.print(" ch_fin_r="); LINK.println(CH_FIN_RIGHT);
  LINK.print("PIN thruster in1="); LINK.print(PIN_THRUSTER_IN1);
  LINK.print(" in2="); LINK.print(PIN_THRUSTER_IN2);
  LINK.print(" pwm="); LINK.println(PIN_THRUSTER_PWM);
  LINK.print("PIN ballast fill="); LINK.print(PIN_BALLAST_FILL);
  LINK.print(" drain="); LINK.print(PIN_BALLAST_DRAIN);
  LINK.print(" level_adc="); LINK.println(PIN_BALLAST_LEVEL_ADC);
  LINK.print("PIN battery_adc="); LINK.println(PIN_BATTERY_ADC);
  LINK.print("PIN depth_adc="); LINK.print(PIN_DEPTH_ADC);
  LINK.print(" i2c_addr=0x"); LINK.println(DEPTH_I2C_ADDR, HEX);
  LINK.print("PIN leak ");
  for (int i = 0; i < NUM_LEAKS; i++) {
    LINK.print(LEAK_PINS[i]);
    if (i < NUM_LEAKS - 1) LINK.print(",");
  }
  LINK.println();
  LINK.print("PIN imu addr=0x"); LINK.println(IMU_I2C_ADDR, HEX);
  LINK.println("OK PINS END");
}

void printHelp()
{
  LINK.println("OK HELP");
  LINK.println("  PING              -> OK PONG");
  LINK.println("  PINS              -> pin map dump");
  LINK.println("  TEST S <ch> <val> -> servo channel 0-3, val -1..1");
  LINK.println("  TEST T <val>      -> thruster -1..1");
  LINK.println("  TEST B fill|drain|stop -> ballast outputs");
  LINK.println("  TEST L            -> read leak sensors");
  LINK.println("  TEST A            -> read all ADC values");
  LINK.println("  B <val>           -> ballast command");
  LINK.println("  S2 y z F fl fr X thr -> sub actuators");
}

void parseTestCommand(char *line)
{
  char sub[8];
  if (sscanf(line, "TEST %7s", sub) != 1) return;

  if (strcmp(sub, "S") == 0 || strncmp(sub, "S", 1) == 0) {
    int ch;
    float val;
    if (sscanf(line, "TEST S %d %f", &ch, &val) == 2 && ch >= 0 && ch <= 3) {
      lastSerialMs = millis();
      setServoChannel((uint8_t)ch, val);
      LINK.print("OK TEST S ch="); LINK.print(ch);
      LINK.print(" val="); LINK.println(val, 3);
    }
    return;
  }
  if (strcmp(sub, "T") == 0 || strncmp(sub, "T", 1) == 0) {
    float val;
    if (sscanf(line, "TEST T %f", &val) == 1) {
      lastSerialMs = millis();
      setThruster(val);
      LINK.print("OK TEST T val="); LINK.println(val, 3);
    }
    return;
  }
  if (strcmp(sub, "B") == 0 || strncmp(sub, "B", 1) == 0) {
    char action[8];
    if (sscanf(line, "TEST B %7s", action) == 1) {
      lastSerialMs = millis();
      if (strcmp(action, "fill") == 0) setBallast(1.0f);
      else if (strcmp(action, "drain") == 0) setBallast(-1.0f);
      else setBallast(0.0f);
      LINK.print("OK TEST B "); LINK.println(action);
    }
    return;
  }
  if (strcmp(sub, "L") == 0) {
    readLeaks();
    LINK.print("OK LEAK");
    for (int i = 0; i < NUM_LEAKS; i++) {
      if (pinValid(LEAK_PINS[i])) {
        LINK.print(" ");
        LINK.print(telemLeaks[i] ? 1 : 0);
      }
    }
    LINK.println();
    return;
  }
  if (strcmp(sub, "A") == 0) {
    LINK.print("OK ADC battery=");
    LINK.print(readAdcVolts(PIN_BATTERY_ADC) * BATTERY_DIVIDER_RATIO, 2);
    LINK.print(" ballast=");
    LINK.print(readAdcVolts(PIN_BALLAST_LEVEL_ADC), 3);
    LINK.print(" depth=");
    LINK.println(readAdcVolts(PIN_DEPTH_ADC), 3);
    return;
  }
}

void parseDiagnostic(char *line)
{
  if (strcmp(line, "PING") == 0) {
    LINK.println("OK PONG");
    return;
  }
  if (strcmp(line, "PINS") == 0) {
    printPinMap();
    return;
  }
  if (strcmp(line, "HELP") == 0 || strcmp(line, "h") == 0) {
    printHelp();
    return;
  }
  if (strncmp(line, "TEST ", 5) == 0) {
    parseTestCommand(line);
    return;
  }
}

void parseLine(char *line)
{
  if (line[0] == '\0') return;

  // Diagnostics first (do not affect sustained control state unless TEST)
  if (strcmp(line, "PING") == 0 || strcmp(line, "PINS") == 0 ||
      strcmp(line, "HELP") == 0 || strcmp(line, "h") == 0 ||
      strncmp(line, "TEST ", 5) == 0) {
    parseDiagnostic(line);
    return;
  }

  if (line[0] == 'B' && (line[1] == ' ' || line[1] == '\0')) {
    parseBallast(line);
    return;
  }
  if (strncmp(line, "S2 ", 3) == 0) {
    parseSubActuators(line);
    return;
  }
  if (line[0] == 'S' && strchr(line, 'D') != NULL && strchr(line, 'T') != NULL) {
    parseCarLine(line);
    return;
  }
}

// Sensors

void initLeakPins()
{
  telemLeakCount = 0;
  for (int i = 0; i < NUM_LEAKS; i++) {
    if (pinValid(LEAK_PINS[i])) {
      pinMode(LEAK_PINS[i], INPUT);
      telemLeakCount++;
    }
  }
}

void readLeaks()
{
  for (int i = 0; i < NUM_LEAKS; i++) {
    if (pinValid(LEAK_PINS[i]))
      telemLeaks[i] = digitalRead(LEAK_PINS[i]) == HIGH;
    else
      telemLeaks[i] = false;
  }
}

void readBattery()
{
  if (!pinValid(PIN_BATTERY_ADC)) return;
  telemBatteryV = readAdcVolts(PIN_BATTERY_ADC) * BATTERY_DIVIDER_RATIO;
}

void readBallastLevel()
{
  if (!pinValid(PIN_BALLAST_LEVEL_ADC)) return;
  float v = readAdcVolts(PIN_BALLAST_LEVEL_ADC);
  reportedBallastLevel = clampf(v / BATTERY_ADC_VREF);  // calibrate to your sensor
}

void readDepth()
{
  if (pinValid(PIN_DEPTH_ADC)) {
    float v = readAdcVolts(PIN_DEPTH_ADC);
    telemDepthM = (v / BATTERY_ADC_VREF) * DEPTH_ADC_MAX_M;
    return;
  }
  // TODO: add I2C depth read when DEPTH_I2C_ADDR != 0
}

// Minimal MPU6050 init + read (no external library)
bool imuWriteByte(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool initImu()
{
  if (IMU_I2C_ADDR == 0) return false;
  if (!imuWriteByte(0x6B, 0x00)) return false;  // wake
  delay(50);
  imuWriteByte(0x1B, 0x08);  // gyro +/-500 deg/s
  imuWriteByte(0x1C, 0x08);  // accel +/-4 g
  return true;
}

void readImu()
{
  if (!imuReady) return;
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return;
  if (Wire.requestFrom((int)IMU_I2C_ADDR, 14) != 14) return;

  int16_t ax = (Wire.read() << 8) | Wire.read();
  int16_t ay = (Wire.read() << 8) | Wire.read();
  int16_t az = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read();  // skip temp
  int16_t gx = (Wire.read() << 8) | Wire.read();
  int16_t gy = (Wire.read() << 8) | Wire.read();
  int16_t gz = (Wire.read() << 8) | Wire.read();

  // Rough attitude from accel; integrate gyro for fast motion (calibrate on bench)
  telemRoll  = atan2f((float)ay, (float)az) * 57.2958f;
  telemPitch = atan2f(-(float)ax, sqrtf((float)ay * ay + (float)az * az)) * 57.2958f;
  telemYaw  += (float)gz / 65.5f * 0.2f;  // 500 dps scale, ~200 ms telem interval
}

void readSensors()
{
  readBattery();
  readBallastLevel();
  readLeaks();
  readDepth();
  readImu();
}

// Telemetry TX

void sendTelemetry()
{
  if (pinValid(PIN_BATTERY_ADC)) {
    LINK.print("TEL battery ");
    LINK.println(telemBatteryV, 2);
  }

  if (imuReady) {
    LINK.print("TEL gyro ");
    LINK.print(telemPitch, 2);
    LINK.print(" ");
    LINK.print(telemRoll, 2);
    LINK.print(" ");
    LINK.println(telemYaw, 2);
  }

  if (pinValid(PIN_DEPTH_ADC) || DEPTH_I2C_ADDR != 0) {
    LINK.print("TEL depth ");
    LINK.println(telemDepthM, 2);
  }

  if (telemLeakCount > 0) {
    LINK.print("TEL leak");
    for (int i = 0; i < NUM_LEAKS; i++) {
      if (pinValid(LEAK_PINS[i])) {
        LINK.print(" ");
        LINK.print(telemLeaks[i] ? 1 : 0);
      }
    }
    LINK.println();
  }

  if (pinValid(PIN_BALLAST_FILL) || pinValid(PIN_BALLAST_DRAIN) || pinValid(PIN_BALLAST_LEVEL_ADC)) {
    LINK.print("TEL ballast ");
    LINK.println(reportedBallastLevel, 3);
  }
}

// Setup and loop

void setup()
{
#if USE_PI_UART
  PiLink.begin(SERIAL_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
#else
  Serial.begin(SERIAL_BAUD);
#endif
  delay(200);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

  // ADC pins
  if (pinValid(PIN_BATTERY_ADC)) pinMode(PIN_BATTERY_ADC, INPUT);
  if (pinValid(PIN_BALLAST_LEVEL_ADC)) pinMode(PIN_BALLAST_LEVEL_ADC, INPUT);
  if (pinValid(PIN_DEPTH_ADC)) pinMode(PIN_DEPTH_ADC, INPUT);

  initLeakPins();

  if (pinValid(PIN_BALLAST_FILL)) {
    pinMode(PIN_BALLAST_FILL, OUTPUT);
    digitalWrite(PIN_BALLAST_FILL, LOW);
  }
  if (pinValid(PIN_BALLAST_DRAIN)) {
    pinMode(PIN_BALLAST_DRAIN, OUTPUT);
    digitalWrite(PIN_BALLAST_DRAIN, LOW);
  }

  if (pinValid(PIN_THRUSTER_IN1)) pinMode(PIN_THRUSTER_IN1, OUTPUT);
  if (pinValid(PIN_THRUSTER_IN2)) pinMode(PIN_THRUSTER_IN2, OUTPUT);
  if (pinValid(PIN_THRUSTER_PWM)) {
    ledcAttach(PIN_THRUSTER_PWM, THRUSTER_PWM_FREQ, THRUSTER_PWM_RES);
    ledcWrite(PIN_THRUSTER_PWM, 0);
  }

  pca9685.begin();
  pca9685.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pca9685.setPWMFreq(50);
  delay(10);
  pcaReady = true;

  setServoChannel(CH_AFT_STEER_Y, 0.0f);
  setServoChannel(CH_AFT_STEER_Z, 0.0f);
  setServoChannel(CH_FIN_LEFT, 0.0f);
  setServoChannel(CH_FIN_RIGHT, 0.0f);

  imuReady = initImu();

  lastSerialMs = millis();
  lastTelemetryMs = millis();

  LINK.println("sub_rc ready - waiting for B / S2 lines from Pi");
  LINK.println("Diagnostics: PING | PINS | TEST S/T/B/L/A | HELP");
  LINK.println("Telemetry: TEL battery | gyro | depth | leak | ballast");
}

void loop()
{
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

  // Thruster safety timeout
  if (millis() - lastSerialMs > SERIAL_TIMEOUT_MS) {
    if (lastThruster != 0.0f) {
      lastThruster = 0.0f;
      setThruster(0.0f);
    }
    if (lastBallastCmd != 0.0f) {
      lastBallastCmd = 0.0f;
      setBallast(0.0f);
    }
  } else {
    setThruster(lastThruster);
  }

  if (millis() - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = millis();
    readSensors();
    sendTelemetry();
  }
}
