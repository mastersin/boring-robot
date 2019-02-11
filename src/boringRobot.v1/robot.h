#ifndef _ROBOT_H
#define _ROBOT_H

void log(const char*);
void log(const char*, int);

extern "C" {
typedef void (*callback)(void);
}

class Drivers;

enum AnalogSensors
{
  ForwardSensorRight,
  ForwardSensorLeft,
  CentralSensorRight,
  CentralSensorLeft,
  BackwardSensorRight,
  BackwardSensorLeft
};

enum ScreenState
{
  RobotStatusInfo,
  EncodersInfo,
  MotorsInfo,
  AnalogInfo1,
  AnalogInfo2,
  ColorInfo,
  LastInfo
};

enum ColorType
{
  NoColor,
  BlackColor,
  BlueColor,
  GreenColor,
  YellowColor,
  RedColor,
  WhiteColor,
  BrownColor
};

class Robot
{
public:
  Robot();
  void init(const char* name);
  void poll();
  void showPoll();

  void setButtonCallbacks(callback cbp, callback cbc = 0, callback cbd = 0);
  void setStatus(const char*);
  void showNextInfo();

  void setPowerA(int);
  void setPowerB(int);
  void setSteeringPower(int, int);

  int colorSensorRed();
  int colorSensorGreen();
  int colorSensorBlue();
  ColorType colorSensor();
  const char *colorSensorName();

  int rotaryButton();

  int analogSensor(AnalogSensors);

private:
  Drivers &drv;
  const char *name;
  bool needUpdateScreen;
  bool needUpdateStatus;
  char status[17];
  ScreenState screenState;

  void showStatus();
  void showEncoders();
  void showMotors();
  void showAnalog1();
  void showAnalog2();
  void showColor();
};

#endif
