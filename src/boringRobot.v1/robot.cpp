#include "robot.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

void log(const char* logLine) {
  Serial.print(millis());
  Serial.print(": ");
  Serial.println(logLine);
}

void log(const char* logLine, int value) {
  Serial.print(millis());
  Serial.print(": ");
  Serial.print(logLine);
  Serial.println(value);
}

class Drivers
{
public:
  Drivers():
    lcd(0x27, 16, 2) {}

  LiquidCrystal_I2C lcd;
};

static Drivers static_drv;

Robot::Robot():
  drv(static_drv)
{
}

void Robot::init(const char* name)
{
  drv.lcd.begin();
  drv.lcd.backlight();
  drv.lcd.print(name);
}

void Robot::poll()
{
}
