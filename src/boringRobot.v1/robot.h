#ifndef _ROBOT_H
#define _ROBOT_H

void log(const char*);
void log(const char*, int);

extern "C" {
typedef void (*callback)(void);
}

class Drivers;

enum ScreenState
{
  RobotStatusInfo,
  EncodersInfo,
  LastInfo
};

class Robot
{
public:
  Robot();
  void init(const char* name);
  void poll();

  void setButtonCallbacks(callback cbp, callback cbc = 0, callback cbd = 0);
  void setStatus(const char*);

private:
  Drivers &drv;
  const char *name;
  bool needUpdateStatus;
  char status[17];
  ScreenState screenState;

  void showStatus();
};

#endif
