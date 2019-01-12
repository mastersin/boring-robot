// Adafruit Motor shield library
// copyright Adafruit Industries LLC, 2009
// this code is public domain, enjoy!

#include <Arduino.h>
#include "AFMotor.h"

AF_DCMotor motor1(3);
AF_DCMotor motor2(1);

void halt() {
    motor1.run(RELEASE);
    motor2.run(RELEASE);
}

void setup() {
  Serial.begin(9600);
  Serial.println("Motor test!");

  motor1.setSpeed(255);  
  motor2.setSpeed(255);  
  halt();
}

void loop() {
    motor1.run(BACKWARD);
    motor2.run(BACKWARD);
    delay(1000);
    halt();
    delay(2000);
}
