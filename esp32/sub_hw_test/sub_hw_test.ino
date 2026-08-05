/*
 * ESP32-S3 Sub Hardware Test (standalone bench firmware)
 *
 * Purpose: test sub electronics without Raspberry Pi control.
 * Flash this, open Serial Monitor @ 115200, use commands below.
 *
 * Pin map matches esp32/sub_rc/sub_rc.ino — edit both if wiring changes.
 *
 * PCA9685 (I2C SDA=8, SCL=9):
 *   ch 0 = aft steer Y   ch 1 = aft steer Z
 *   ch 2 = fin left      ch 3 = fin right
 * Thruster L298N: IN1=4, IN2=5, PWM=6
 * Ballast: fill=10, drain=11
 * Leak sensors: GPIO 12-15 (digital, HIGH=leak)
 * ADC: battery=1, ballast level=2, depth=3
 */

#if defined(ARDUINO_ARCH_ESP32) && !defined(ESP32)
#define ESP32 1
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <string.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define CH_AFT_Y 0
#define CH_AFT_Z 1
#define CH_FIN_L 2
#define CH_FIN_R 3
#define PIN_THR_IN1 4
#define PIN_THR_IN2 5
#define PIN_THR_PWM 6
#define PIN_BALLAST_FILL 10
#define PIN_BALLAST_DRAIN 11
#define PIN_BATTERY_ADC 1
#define PIN_BALLAST_ADC 2
#define PIN_DEPTH_ADC 3
#define PIN_LEAK_1 12
#define PIN_LEAK_2 13
#define PIN_LEAK_3 14
#define PIN_LEAK_4 15

#define SERVO_MIN 205
#define SERVO_MAX 410
#define SERVO_CENTER 307
#define PCA9685_OSC_FREQ 25000000
#define THR_PWM_FREQ 5000
#define THR_PWM_RES 8
#define THR_MAX_PWM 255
#define THR_MIN_START 90

Adafruit_PWMServoDriver pca = Adafruit_PWMServoDriver(0x40);

static const int LEAK_PINS[] = { PIN_LEAK_1, PIN_LEAK_2, PIN_LEAK_3, PIN_LEAK_4 };
static const char *CH_NAMES[] = { "aft_y", "aft_z", "fin_l", "fin_r" };

static float clampf(float v) {
  if (v < -1.0f) return -1.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

uint16_t servoTick(float v) {
  v = clampf(v);
  float half = (SERVO_MAX - SERVO_MIN) / 2.0f;
  return (uint16_t)(SERVO_CENTER + (v * half));
}

void setServoCh(int ch, float v) {
  pca.setPWM(ch, 0, servoTick(v));
  Serial.print("Servo ch"); Serial.print(ch);
  Serial.print(" ("); Serial.print(CH_NAMES[ch]); Serial.print(") = ");
  Serial.println(v, 2);
}

void setThruster(float v) {
  v = clampf(v);
  int pwm = (int)(fabsf(v) * THR_MAX_PWM);
  if (pwm > 0) pwm = max(THR_MIN_START, pwm);
  pwm = min(THR_MAX_PWM, pwm);

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
  Serial.print("Thruster = "); Serial.println(v, 2);
}

void setBallast(const char *action) {
  bool fill = strcmp(action, "fill") == 0;
  bool drain = strcmp(action, "drain") == 0;
  digitalWrite(PIN_BALLAST_FILL, fill ? HIGH : LOW);
  digitalWrite(PIN_BALLAST_DRAIN, drain ? HIGH : LOW);
  Serial.print("Ballast: "); Serial.println(action);
}

void readLeaks() {
  Serial.print("Leaks: ");
  for (int i = 0; i < 4; i++) {
    bool leak = digitalRead(LEAK_PINS[i]) == HIGH;
    Serial.print("L"); Serial.print(i + 1);
    Serial.print("="); Serial.print(leak ? "LEAK" : "ok");
    if (i < 3) Serial.print("  ");
  }
  Serial.println();
}

void readAdcs() {
  Serial.print("ADC battery=");
  Serial.print(analogRead(PIN_BATTERY_ADC) * 3.3f / 4095.0f * 11.0f, 2);
  Serial.print("V  ballast=");
  Serial.print(analogRead(PIN_BALLAST_ADC) * 3.3f / 4095.0f, 3);
  Serial.print("V  depth=");
  Serial.println(analogRead(PIN_DEPTH_ADC) * 3.3f / 4095.0f, 3);
}

void printHelp() {
  Serial.println();
  Serial.println("=== sub_hw_test commands ===");
  Serial.println("h        help");
  Serial.println("p        print pin map");
  Serial.println("0        stop all (servos center, thruster off, ballast stop)");
  Serial.println("1-4      servo ch 0-3 pulse to +1.0 for 1.2s then center");
  Serial.println("f / r    thruster forward / reverse pulse (1.2s)");
  Serial.println("b fill / b drain / b stop   ballast outputs");
  Serial.println("l        read leak sensors");
  Serial.println("a        read ADC values");
  Serial.println("s        sweep all 4 servos");
  Serial.println();
}

void printPins() {
  Serial.println("--- Pin map (matches sub_rc.ino) ---");
  Serial.println("I2C SDA=8 SCL=9  PCA9685 @ 0x40");
  Serial.println("Servos: ch0=aft_y ch1=aft_z ch2=fin_l ch3=fin_r");
  Serial.println("Thruster: IN1=4 IN2=5 PWM=6");
  Serial.println("Ballast: fill=10 drain=11 level_adc=2");
  Serial.println("Battery adc=1  Depth adc=3");
  Serial.println("Leaks: 12,13,14,15");
  Serial.println("Pi UART (sub_rc only): RX=18 TX=17");
}

void servoPulse(int ch) {
  setServoCh(ch, 1.0f);
  delay(1200);
  setServoCh(ch, 0.0f);
}

void thrPulse(float v) {
  setThruster(v);
  delay(1200);
  setThruster(0.0f);
}

void servoSweep() {
  Serial.println("Servo sweep all channels...");
  for (int ch = 0; ch < 4; ch++) {
    for (float v = -1.0f; v <= 1.001f; v += 0.25f) {
      setServoCh(ch, v);
      delay(150);
    }
    setServoCh(ch, 0.0f);
  }
  Serial.println("Sweep done");
}

void stopAll() {
  for (int ch = 0; ch < 4; ch++) setServoCh(ch, 0.0f);
  setThruster(0.0f);
  setBallast("stop");
  Serial.println("All stopped");
}

void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA, I2C_SCL);
  pca.begin();
  pca.setOscillatorFrequency(PCA9685_OSC_FREQ);
  pca.setPWMFreq(50);
  delay(10);

  pinMode(PIN_BATTERY_ADC, INPUT);
  pinMode(PIN_BALLAST_ADC, INPUT);
  pinMode(PIN_DEPTH_ADC, INPUT);
  for (int i = 0; i < 4; i++) pinMode(LEAK_PINS[i], INPUT);

  pinMode(PIN_BALLAST_FILL, OUTPUT);
  pinMode(PIN_BALLAST_DRAIN, OUTPUT);
  digitalWrite(PIN_BALLAST_FILL, LOW);
  digitalWrite(PIN_BALLAST_DRAIN, LOW);

  pinMode(PIN_THR_IN1, OUTPUT);
  pinMode(PIN_THR_IN2, OUTPUT);
  ledcAttach(PIN_THR_PWM, THR_PWM_FREQ, THR_PWM_RES);
  ledcWrite(PIN_THR_PWM, 0);

  stopAll();
  Serial.println("sub_hw_test ready");
  printHelp();
}

void loop() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char c = line.charAt(0);

  if (line == "h" || line == "help") printHelp();
  else if (line == "p" || line == "pins") printPins();
  else if (line == "0" || line == "stop") stopAll();
  else if (line == "1") servoPulse(0);
  else if (line == "2") servoPulse(1);
  else if (line == "3") servoPulse(2);
  else if (line == "4") servoPulse(3);
  else if (line == "f") thrPulse(+0.5f);
  else if (line == "r") thrPulse(-0.5f);
  else if (line.startsWith("b ")) setBallast(line.substring(2).c_str());
  else if (line == "l") readLeaks();
  else if (line == "a") readAdcs();
  else if (line == "s") servoSweep();
  else {
    Serial.print("Unknown: ");
    Serial.println(line);
    printHelp();
  }
}
