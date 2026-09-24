#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// PCA9685 pulse limits
#define SERVO_MIN 150
#define SERVO_MAX 600

void setup() {
  Serial.begin(115200);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Start servo controller
  pwm.begin();
  pwm.setPWMFreq(50);

  delay(500);

  Serial.println("Servo controller started");
}

void loop() {

  // Channels 1 to 5
  for (int ch = 1; ch <= 5; ch++) {

    Serial.print("Moving servo ");
    Serial.println(ch);

    // Left -> right
    for (int pos = SERVO_MIN; pos <= SERVO_MAX; pos += 5) {
      pwm.setPWM(ch, 0, pos);
      delay(10);
    }

    // Right -> left
    for (int pos = SERVO_MAX; pos >= SERVO_MIN; pos -= 5) {
      pwm.setPWM(ch, 0, pos);
      delay(10);
    }
  }
}