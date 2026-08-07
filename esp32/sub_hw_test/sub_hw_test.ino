// ESP32-S3 — Sub bench hardware test (no Pi required)
// Open Serial Monitor @ 115200 and use commands below to confirm wiring.

#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define CH_AFT_Y 0
#define CH_AFT_Z 1
#define CH_FIN_L 2
#define CH_FIN_R 3
#define PIN_THR_IN1 4
#define PIN_THR_IN2 5
#define PIN_THR_PWM 6
#define PIN_BALLAST_INA  10
#define PIN_BALLAST_INB  11
#define PIN_BALLAST_ADC  2
#define PIN_BATTERY_ADC 1
#define PIN_DEPTH_ADC 3
#define PIN_LEAK_0 12
#define PIN_LEAK_1 13
#define PIN_LEAK_2 14
#define PIN_LEAK_3 15
#define SERVO_MIN 205
#define SERVO_MAX 410
#define SERVO_CENTER 307
#define MOTOR_MAX 255
#define MOTOR_MIN_START 90

Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(0x40);

static float clampf(float v) {
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

static uint16_t servoTick(float v) {
  v = clampf(v);
  return (uint16_t)(SERVO_CENTER + v * (SERVO_MAX - SERVO_MIN) / 2.0f);
}

static void setServo(int ch, float v) {
  pca9685.setPWM(ch, 0, servoTick(v));
}

static void setThruster(float v) {
  int pwm = (int)(fabsf(clampf(v)) * MOTOR_MAX);
  if (pwm > 0) pwm = max(MOTOR_MIN_START, pwm);
  if (v > 0.02f) {
    digitalWrite(PIN_THR_IN1, HIGH);
    digitalWrite(PIN_THR_IN2, LOW);
    ledcWrite(PIN_THR_PWM, pwm);
  } else if (v < -0.02f) {
    digitalWrite(PIN_THR_IN1, LOW);
    digitalWrite(PIN_THR_IN2, HIGH);
    ledcWrite(PIN_THR_PWM, pwm);
  } else {
    digitalWrite(PIN_THR_IN1, LOW);
    digitalWrite(PIN_THR_IN2, LOW);
    ledcWrite(PIN_THR_PWM, 0);
  }
}

static void setBallast(const char *mode) {
  bool fill = strcmp(mode, "fill") == 0;
  bool drain = strcmp(mode, "drain") == 0;
  if (fill) {
    digitalWrite(PIN_BALLAST_INA, HIGH);
    digitalWrite(PIN_BALLAST_INB, LOW);
  } else if (drain) {
    digitalWrite(PIN_BALLAST_INA, LOW);
    digitalWrite(PIN_BALLAST_INB, HIGH);
  } else {
    digitalWrite(PIN_BALLAST_INA, LOW);
    digitalWrite(PIN_BALLAST_INB, LOW);
  }
}

static void printPins() {
  Serial.println("=== sub pin map ===");
  Serial.printf("I2C SDA=%d SCL=%d  PCA9685 ch0=aftY ch1=aftZ ch2=finL ch3=finR\n", I2C_SDA, I2C_SCL);
  Serial.printf("Thruster IN1=%d IN2=%d PWM=%d\n", PIN_THR_IN1, PIN_THR_IN2, PIN_THR_PWM);
  Serial.printf("Ballast INA=%d INB=%d adc=%d\n", PIN_BALLAST_INA, PIN_BALLAST_INB, PIN_BALLAST_ADC);
  Serial.printf("Battery ADC=%d  Depth ADC=%d\n", PIN_BATTERY_ADC, PIN_DEPTH_ADC);
  Serial.printf("Leaks GPIO %d,%d,%d,%d\n", PIN_LEAK_0, PIN_LEAK_1, PIN_LEAK_2, PIN_LEAK_3);
}

static void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  h/help  p/pins  0/stop");
  Serial.println("  1-4     servo pulse ch0-3 (+0.5 for 1.2s)");
  Serial.println("  f/r     thruster forward/reverse pulse");
  Serial.println("  b fill|drain|stop");
  Serial.println("  l       read leak GPIO");
  Serial.println("  a       read ADC (battery/ballast/depth)");
  Serial.println("  s       servo sweep all channels");
  Serial.println();
}

static void servoPulse(int ch) {
  Serial.printf("Servo ch%d pulse +0.5\n", ch);
  setServo(ch, 0.5f);
  delay(1200);
  setServo(ch, 0.0f);
}

static void thrPulse(float v) {
  Serial.printf("Thruster %.2f\n", v);
  setThruster(v);
  delay(1200);
  setThruster(0);
}

static void readLeaks() {
  Serial.printf("Leaks: %d %d %d %d\n",
    digitalRead(PIN_LEAK_0), digitalRead(PIN_LEAK_1),
    digitalRead(PIN_LEAK_2), digitalRead(PIN_LEAK_3));
}

static void readAdc() {
  Serial.printf("ADC battery=%d ballast=%d depth=%d\n",
    analogRead(PIN_BATTERY_ADC), analogRead(PIN_BALLAST_ADC), analogRead(PIN_DEPTH_ADC));
}

static void servoSweep() {
  for (float v = -1.0f; v <= 1.01f; v += 0.25f) {
    for (int ch = 0; ch <= 3; ch++) setServo(ch, v);
    delay(150);
  }
  for (int ch = 0; ch <= 3; ch++) setServo(ch, 0);
}

void setup() {
  Serial.begin(115200);
  delay(200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pca9685.begin();
  pca9685.setPWMFreq(50);
  pinMode(PIN_THR_IN1, OUTPUT);
  pinMode(PIN_THR_IN2, OUTPUT);
  pinMode(PIN_BALLAST_INA, OUTPUT);
  pinMode(PIN_BALLAST_INB, OUTPUT);
  pinMode(PIN_LEAK_0, INPUT);
  pinMode(PIN_LEAK_1, INPUT);
  pinMode(PIN_LEAK_2, INPUT);
  pinMode(PIN_LEAK_3, INPUT);
  ledcAttach(PIN_THR_PWM, 5000, 8);
  analogReadResolution(12);
  setThruster(0);
  setBallast("stop");
  for (int ch = 0; ch <= 3; ch++) setServo(ch, 0);
  Serial.println("sub_hw_test ready");
  printHelp();
  printPins();
}

void loop() {
  if (!Serial.available()) return;
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "h" || cmd == "help") printHelp();
  else if (cmd == "p" || cmd == "pins") printPins();
  else if (cmd == "0" || cmd == "stop") { setThruster(0); setBallast("stop"); for (int ch=0;ch<=3;ch++) setServo(ch,0); Serial.println("All stop"); }
  else if (cmd == "1") servoPulse(0);
  else if (cmd == "2") servoPulse(1);
  else if (cmd == "3") servoPulse(2);
  else if (cmd == "4") servoPulse(3);
  else if (cmd == "f") thrPulse(0.4f);
  else if (cmd == "r") thrPulse(-0.4f);
  else if (cmd.startsWith("b ")) setBallast(cmd.substring(2).c_str());
  else if (cmd == "l") readLeaks();
  else if (cmd == "a") readAdc();
  else if (cmd == "s") servoSweep();
  else { Serial.print("Unknown: "); Serial.println(cmd); printHelp(); }
}
