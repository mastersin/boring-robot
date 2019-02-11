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

#define DIAMETER 65
#define STEP_PER_COUNT (DIAMETER * 314159 / 32 / 100)

static const int16_t MAX_STEERING = 100;
static const int16_t AVR_STEERING = MAX_STEERING / 2;
static const int16_t MIN_STEERING = -MAX_STEERING;

static const int16_t MAX_POWER = 100;
static const int16_t MIN_POWER = -MAX_POWER;

class Motor
{
public:
  Motor(uint8_t _dir, uint8_t _pwm): dir(_dir), pwm(_pwm), power(0)
  {
    pinMode(dir, OUTPUT);
    pinMode(pwm, OUTPUT);
    digitalWrite(dir, LOW);
    analogWrite(pwm, 0);
  }
  void operator= (int newPower) { set(newPower); }
  const int& operator()() const { return power; }
  void set(int newPower);

private:
  uint8_t dir;
  uint8_t pwm;

  int power;
};

void Motor::set(int newPower)
{
  int setPower = newPower;

  if (newPower > MAX_POWER)
    newPower = MAX_POWER;
  if (newPower < MIN_POWER)
    newPower = MIN_POWER;

  if (newPower >= 0) {
    if(power < 0)
      digitalWrite(dir, HIGH);
  } else {
    if (power >= 0)
      digitalWrite(dir, LOW);
    setPower = -newPower;
  }

  power = newPower;
  setPower = setPower * 255 / MAX_POWER;
  analogWrite(pwm, setPower);
}

class Encoder
{
public:
  Encoder(): counter(0), prev_counter(0), diff_counter(0) {}
  void operator++() {
    unsigned long curr_millis = millis();
    if (curr_millis - last_millis < 10)
      return;
    last_millis = curr_millis;
    ++counter;
  }
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
  unsigned long diff()
  {
    return diff_counter;
  }

private:
  volatile unsigned long counter;
  unsigned long prev_counter;
  unsigned long diff_counter;
  unsigned long last_millis;
};

void Encoder::poll()
{
  unsigned long curr_counter = counter;
  unsigned long diff = curr_counter - prev_counter;
  if (curr_counter != prev_counter) {
    prev_counter = curr_counter;
  }
  diff_counter = diff;
}

void interruptHandlerEncoderA();
void interruptHandlerEncoderB();

class Drivers
{
public:
  Drivers():
    lcd(0x27, 16, 2), button(BUTTON_PIN, true),
    motorA(MOTORA_DIR_PIN, MOTORA_PWM_PIN),
    motorB(MOTORB_DIR_PIN, MOTORB_PWM_PIN)
  {
    pinMode(ENCODERA_PIN, INPUT_PULLUP);
    pinMode(ENCODERB_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODERA_PIN), interruptHandlerEncoderA, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODERB_PIN), interruptHandlerEncoderB, RISING);
  }

  LiquidCrystal_I2C lcd;
  OneButton button;
  Encoder encoderA;
  Encoder encoderB;
  Motor motorA;
  Motor motorB;
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

static char showBuffer[17];

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

Robot::Robot():
  drv(static_drv), name("unknownRobot"), needUpdateScreen(false), needUpdateStatus(false), screenState(RobotStatusInfo) {}

void Robot::init(const char* newName)
{
  name = newName;
  drv.lcd.begin();
  drv.lcd.backlight();
  prepareString(showBuffer, name);
  drv.lcd.print(showBuffer);
}

void Robot::showStatus()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, name);
    drv.lcd.print(showBuffer);
    needUpdateScreen = false;
    needUpdateStatus = true;
  }

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
  drv.encoderA.poll();
  drv.encoderB.poll();

  switch(screenState) {
    default:
    case RobotStatusInfo:
      showStatus();
      break;
    case EncodersInfo:
      showEncoders();
      break;
    case MotorsInfo:
      showMotors();
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

void Robot::setStatus(const char *newStatus)
{
  prepareString(status, newStatus);
  needUpdateStatus = true;
}

void Robot::showEncoders()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, "EncA = ");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(7,0);
    drv.lcd.print(drv.encoderA());
    drv.lcd.print(' ');
    drv.lcd.print(drv.encoderA.diff());

    drv.lcd.setCursor(0,1);
    prepareString(showBuffer, "EncB = ");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(7,1);
    drv.lcd.print(drv.encoderB());
    drv.lcd.print(' ');
    drv.lcd.print(drv.encoderB.diff());

    needUpdateScreen = false;
  }
}

void Robot::showMotors()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, "MotA = ");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(7,0);
    drv.lcd.print(drv.motorA());

    drv.lcd.setCursor(0,1);
    prepareString(showBuffer, "MotB = ");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(7,1);
    drv.lcd.print(drv.motorB());

    needUpdateScreen = false;
  }
}
void Robot::showNextInfo()
{
  screenState = screenState + 1;
  if (screenState >= LastInfo)
    screenState = RobotStatusInfo;
  needUpdateScreen = true;
}

void Robot::showPoll()
{
  if(screenState == EncodersInfo || screenState == MotorsInfo)
    needUpdateScreen = true;
}

void Robot::setPowerA(int power)
{
  drv.motorA = power;
}

void Robot::setPowerB(int power)
{
  drv.motorB = power;
}

void Robot::setSteeringPower(int steering, int power)
{
  int powerA = power;
  int powerB = power;

  if (steering > MAX_STEERING)
    steering = MAX_STEERING;
  if (steering < MIN_STEERING)
    steering = MIN_STEERING;

  if (steering > 0) {
    int k = AVR_STEERING - steering;
    powerA = powerA * k / AVR_STEERING;
  }

  if (steering < 0) {
    int k = AVR_STEERING - steering;
    powerB = powerB * k / AVR_STEERING;
  }

  drv.motorA = powerA;
  drv.motorB = powerB;
}
