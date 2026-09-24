// Isolation = working sweep on GPIO 8/9 I2C (same as sub_rc).
// Flash:  bash esp32/upload_from_pi.sh --sketch subrcservoisolate

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

  Serial.println("Servo controller started");
  Serial.println("I2C SDA=8 SCL=9");
}

void loop() {
  for (int ch = 1; ch <= 5; ch++) {
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
