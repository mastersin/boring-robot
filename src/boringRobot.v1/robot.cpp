#include "robot.h"

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <OneButton.h>
#include <IoAbstraction.h>
#include <AnalogScanner.h>

#define BUTTON_PIN     34
#define MOTORA_DIR_PIN 13
#define MOTORB_DIR_PIN 12
#define MOTORA_PWM_PIN 3
#define MOTORB_PWM_PIN 11
#define ENCODERA_PIN   18
#define ENCODERB_PIN   19
#define COLOR_LED_PIN  28
#define COLOR_S0_PIN   29
#define COLOR_S1_PIN   30
#define COLOR_S2_PIN   31
#define COLOR_S3_PIN   32
#define COLOR_OUT_PIN  33

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

class Color
{
public:
  Color(uint8_t _led, uint8_t _s0, uint8_t _s1, uint8_t _s2, uint8_t _s3, uint8_t _out):
    led(_led), s0(_s0), s1(_s1), s2(_s2), s3(_s3), out(_out), red(0), blue(0), green(0)
  {
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(out, INPUT);

    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    digitalWrite(led, LOW);
  }

  ColorType operator()() { return 0; }
  void poll();

  int red;
  int green;
  int blue;

private:
  uint8_t led, s0, s1, s2, s3, out;

};

void Color::poll()
{
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);
  red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s3, HIGH);
  blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
  digitalWrite(s2, HIGH);
  green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void interruptHandlerEncoderA();
void interruptHandlerEncoderB();

int analogScanOrder[] = {A0, A1, A2, A3, A4, A5};
const int ANALOG_SCAN_COUNT = sizeof(analogScanOrder) / sizeof(analogScanOrder[0]);

class Drivers
{
public:
  Drivers():
    lcd(0x27, 16, 2), button(BUTTON_PIN, true),
    motorA(MOTORA_DIR_PIN, MOTORA_PWM_PIN),
    motorB(MOTORB_DIR_PIN, MOTORB_PWM_PIN),
    color(COLOR_LED_PIN, COLOR_S0_PIN, COLOR_S1_PIN, COLOR_S2_PIN, COLOR_S3_PIN, COLOR_OUT_PIN)
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
  AnalogScanner scanner;
  Color color;
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

  drv.scanner.setScanOrder(ANALOG_SCAN_COUNT, analogScanOrder);
  drv.scanner.beginScanning();

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
  //drv.color.poll();

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
    case AnalogInfo1:
      showAnalog1();
      break;
    case AnalogInfo2:
      showAnalog2();
      break;
    case ColorInfo:
      showColor();
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

void Robot::showAnalog1()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, "");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(0,1);
    drv.lcd.print(showBuffer);

    drv.lcd.setCursor(2,0);
    drv.lcd.print(analogSensor(ForwardSensorLeft));
    drv.lcd.setCursor(10,0);
    drv.lcd.print(analogSensor(ForwardSensorRight));

    drv.lcd.setCursor(1,1);
    drv.lcd.print(analogSensor(CentralSensorLeft));
    drv.lcd.setCursor(11,1);
    drv.lcd.print(analogSensor(CentralSensorRight));

    needUpdateScreen = false;
  }
}

void Robot::showAnalog2()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, "");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(0,1);
    drv.lcd.print(showBuffer);

    drv.lcd.setCursor(1,0);
    drv.lcd.print(analogSensor(CentralSensorLeft));
    drv.lcd.setCursor(11,0);
    drv.lcd.print(analogSensor(CentralSensorRight));

    drv.lcd.setCursor(0,1);
    drv.lcd.print(analogSensor(BackwardSensorLeft));
    drv.lcd.setCursor(12,1);
    drv.lcd.print(analogSensor(BackwardSensorRight));

    needUpdateScreen = false;
  }
}

void Robot::showColor()
{
  if (needUpdateScreen)
  {
    drv.lcd.setCursor(0,0);
    prepareString(showBuffer, "R:      G:      ");
    drv.lcd.print(showBuffer);
    drv.lcd.setCursor(0,1);
    prepareString(showBuffer, "B:              ");
    drv.lcd.print(showBuffer);

    drv.lcd.setCursor(3,0);
    drv.lcd.print(colorSensorRed());
    drv.lcd.setCursor(11,0);
    drv.lcd.print(colorSensorGreen());

    drv.lcd.setCursor(3,1);
    drv.lcd.print(colorSensorBlue());
    drv.lcd.setCursor(8,1);
    drv.lcd.print(colorSensorName());

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
  if(screenState == EncodersInfo || screenState == MotorsInfo || screenState == AnalogInfo1 || screenState == AnalogInfo2)
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

int Robot::analogSensor(AnalogSensors sensor)
{
  return drv.scanner.getValue(analogScanOrder[sensor]);
}

int Robot::colorSensorRed()
{
  return drv.color.red;
}

int Robot::colorSensorGreen()
{
  return drv.color.green;
}

int Robot::colorSensorBlue()
{
  return drv.color.blue;
}

ColorType Robot::colorSensor()
{
  return drv.color();
}

const char *colorNames[]
{
  "unknown",
  "black",
  "blue",
  "green",
  "yellow",
  "red",
  "white",
  "brown"
};

const char *Robot::colorSensorName()
{
  return colorNames[colorSensor()];
}
