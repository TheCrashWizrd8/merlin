// Bench PCA9685 sweep — same I2C/ticks as sub_rc.ino (0x40, SDA=8, SCL=9, 150..600).
// Production sub_rc drives fin servos on PCA headers 2–5.

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define I2C_SDA 8
#define I2C_SCL 9
#define PCA9685_ADDR 0x40

#define SERVO_MIN 150
#define SERVO_MAX 600

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PCA9685_ADDR);

static uint16_t pulseFromAngle(int angleDeg) {
  if (angleDeg < 0) angleDeg = 0;
  if (angleDeg > 180) angleDeg = 180;
  return (uint16_t)map(angleDeg, 0, 180, SERVO_MIN, SERVO_MAX);
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);
  Serial.println("Fin sweep @0x40 SDA=8 SCL=9 — PCA channels 2-5");
}

void loop() {
  for (int angle = 0; angle <= 180; angle++) {
    uint16_t pulse = pulseFromAngle(angle);
    for (int ch = 2; ch <= 5; ch++) {
      pwm.setPWM(ch, 0, pulse);
    }
    delay(10);
  }
  for (int angle = 180; angle >= 0; angle--) {
    uint16_t pulse = pulseFromAngle(angle);
    for (int ch = 2; ch <= 5; ch++) {
      pwm.setPWM(ch, 0, pulse);
    }
    delay(10);
  }
}
