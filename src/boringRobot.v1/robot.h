#ifndef _ROBOT_H
#define _ROBOT_H

#define BUTTON_PIN     34
#define MOTORA_DIR_PIN 13
#define MOTORB_DIR_PIN 12
#define MOTORA_PWM_PIN 3
#define MOTORB_PWM_PIN 11
#define ENCODERA_PIN   18
#define ENCODERB_PIN   19

void log(const char*);
void log(const char*, int);

class Drivers;

class Robot
{
public:
  Robot();
  void init(const char* name);
  void poll();

private:
  Drivers &drv;
};

#endif
