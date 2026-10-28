// Isolation sweep — fin headers 2–5 (matches sub_rc.ino)

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

#define SERVO_MIN 150
#define SERVO_MAX 600

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(500);
  Serial.println("Servo sweep ch2-5 @0x40 SDA=8 SCL=9");
}

void loop() {
  for (int ch = 2; ch <= 5; ch++) {
    Serial.print("Moving servo ");
    Serial.println(ch);
    for (int pos = SERVO_MIN; pos <= SERVO_MAX; pos += 5) {
      pwm.setPWM(ch, 0, pos);
      delay(10);
    }
    for (int pos = SERVO_MAX; pos >= SERVO_MIN; pos -= 5) {
      pwm.setPWM(ch, 0, pos);
      delay(10);
    }
  }
}
