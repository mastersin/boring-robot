#include "robot.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <IoAbstraction.h>

#define BUTTON_PIN     34
#define MOTORA_DIR_PIN 13
#define MOTORB_DIR_PIN 12
#define MOTORA_PWM_PIN 3
#define MOTORB_PWM_PIN 11
#define ENCODERA_PIN   18
#define ENCODERB_PIN   19

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

//void interruptHandlerEncoderA();
//void interruptHandlerEncoderB();

#define DIAMETER 65
#define STEP_PER_COUNT (DIAMETER * 314159 / 32 / 100)

class Encoder
{
public:
  Encoder(): counter(0), prev_counter(0), diff_counter(0) {}
  void operator++() { ++counter; }
  const unsigned long& operator()() const { return prev_counter; }
  const unsigned long& operator*() const { return diff_counter; }
  void poll();

  unsigned long length()
  {
    return (counter * STEP_PER_COUNT) / 1000;
  }
  unsigned long velocity()
  {
    return (diff_counter * STEP_PER_COUNT) / 1000;
  }

private:
  volatile unsigned long counter;
  unsigned long prev_counter;
  unsigned long diff_counter;
};

void Encoder::poll()
{
  unsigned long curr_counter = counter;
  if (curr_counter != prev_counter) {
    unsigned long diff = curr_counter - prev_counter;
    prev_counter = curr_counter;
    diff_counter = diff;
  }
}

void interruptHandlerEncoderA();
void interruptHandlerEncoderB();

class Drivers
{
public:
  Drivers():
    lcd(0x27, 16, 2), button(BUTTON_PIN, true)
  {
    pinMode(ENCODERA_PIN, INPUT_PULLUP);
    pinMode(ENCODERB_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODERA_PIN), interruptHandlerEncoderA, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODERB_PIN), interruptHandlerEncoderB, CHANGE);
  }

  LiquidCrystal_I2C lcd;
  OneButton button;
  Encoder encoderA;
  Encoder encoderB;
};

static Drivers static_drv;

void interruptHandlerEncoderA()
{
  ++static_drv.encoderA;
}

void interruptHandlerEncoderB()
{
  ++static_drv.encoderB;
}

Robot::Robot():
  drv(static_drv), name(name), needUpdateStatus(false), screenState(RobotStatusInfo) {}

void Robot::init(const char* name)
{
  drv.lcd.begin();
  drv.lcd.backlight();
  drv.lcd.print(name);
}

void Robot::showStatus()
{
  if (needUpdateStatus)
  {
    drv.lcd.setCursor(0,1);
    drv.lcd.print(status);
    needUpdateStatus = false;
  }
}

void Robot::poll()
{
  drv.button.tick();

  switch(screenState) {
    default:
    case RobotStatusInfo:
      showStatus();
      break;
    case EncodersInfo:
      break;
  }
}

void Robot::setButtonCallbacks(callback cbp, callback cbc, callback cbd)
{
  if (cbp)
    drv.button.attachLongPressStart(cbp);
  if (cbc)
    drv.button.attachClick(cbc);
  if (cbd)
    drv.button.attachDoubleClick(cbd);
}

void prepareString(char *data, const char *newData)
{
  strncpy (data, newData, 16);
  uint8_t newLen = strlen(newData);
  if (newLen < 16)
  {
    for (uint8_t i = newLen; i < 16; i++)
      data[i] = ' ';
  }
  data[16] = 0;
}

void Robot::setStatus(const char *newStatus)
{
  prepareString(status, newStatus);
  needUpdateStatus = true;
}
