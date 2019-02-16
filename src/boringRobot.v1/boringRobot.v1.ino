#include <IoAbstraction.h>
#include "robot.h"

IoAbstractionRef arduinoIo = ioUsingArduino();

char slotString[20] = { 0 };

int pulseTask = -1;
int pollTask = -1;
int programTask = -1;
int colorSensorTask = -1;

Robot robot;
static const char *name = "boringRobot-v1.0";

void setup() {
  Serial.begin(115200);
  while (!Serial);

  log("Setup boringRobot");

  taskManager.scheduleOnce(1000, oneSecondsStartUp);
  pulseTask = taskManager.scheduleFixedRate(1, oneSecondPulse, TIME_SECONDS);

  taskManager.scheduleFixedRate(5, [] { log(taskManager.checkAvailableSlots(slotString)); }, TIME_SECONDS);
}


static int BASE_POWER = 15;
static bool programStarted = false;
void program()
{
  if (!programStarted) {
    robot.setPowerA(0);
    robot.setPowerB(0);
    return;
  }

  int fr = robot.analogSensor(ForwardSensorRight);
  int fl = robot.analogSensor(ForwardSensorLeft);
  int br = robot.analogSensor(BackwardSensorRight);
  int bl = robot.analogSensor(BackwardSensorLeft);
  int r = robot.analogSensor(CentralSensorRight);
  int l = robot.analogSensor(CentralSensorLeft);

  int er = l - r;
  int k1 = 1;
  int k2 = 20;
  int fs = fl + fr;
  if (fs < 1000)
    k2 = 30;

  int lp = BASE_POWER + er * k1 / k2;
  int rp = BASE_POWER - er * k1 / k2;

  if (lp < 0) lp = 0;
  if (rp < 0) rp = 0;
  if (lp > 60) lp = 60;
  if (rp > 60) rp = 60;

  robot.setPowerA(lp);
  robot.setPowerB(rp);
}

void startProgram()
{
  if (programTask < 0) {
    programTask = taskManager.scheduleFixedRate(1, program);
    log("Start boringRobot, programTask = ", programTask);
    programStarted = true;
    robot.setStatus("Started");
  }
}

void startStopProgram()
{
  if (programTask < 0) {
    startProgram();
    return;
  }

  if (!programStarted) {
    log("Restart boringRobot");
    programStarted = true;
    robot.setStatus("Restarted");
  } else {
    log("Stop boringRobot");
    programStarted = false;
    robot.setStatus("Stoped");
  }

  log(taskManager.checkAvailableSlots(slotString));
}

void nextInfo()
{
  robot.showNextInfo();
}

void oneSecondPulse() {
  log("One second pulse");

  int fr = robot.analogSensor(ForwardSensorRight);
  int fl = robot.analogSensor(ForwardSensorLeft);
  int br = robot.analogSensor(BackwardSensorRight);
  int bl = robot.analogSensor(BackwardSensorLeft);
  int r = robot.analogSensor(CentralSensorRight);
  int l = robot.analogSensor(CentralSensorLeft);
  Serial.print(bl);
  Serial.print(" -> ");
  Serial.print(l);
  Serial.print(" - ");
  Serial.print(fl);
  Serial.print(" : ");
  Serial.print(fr);
  Serial.print(" - ");
  Serial.print(r);
  Serial.print(" <- ");
  Serial.print(br);
  Serial.print(" : ");
  Serial.print(bl - br);
  Serial.print(" + ");
  Serial.println(l - r);

  int er = l - r;
  int k1 = 1;
  int k2 = 20;
  int fs = fl + fr;
  if (fs < 1000)
    k2 = 30;

  Serial.print(BASE_POWER + er * k1 / k2);
  Serial.print(" | ");
  Serial.println(BASE_POWER - er * k1 / k2);

  robot.showPoll();
}

void oneSecondsStartUp() {
  log("Initial boringRobot");

  robot.init("boringRobot-v1.0");
  robot.setButtonCallbacks(startStopProgram, startProgram, nextInfo);
  robot.setStatus("Ready");

  pollTask = taskManager.scheduleFixedRate(100, [] {
    robot.poll();
  });
  colorSensorTask = taskManager.scheduleFixedRate(robot.colorInterval(), [] {
    robot.colorPoll();
  });
}

void loop() {
  taskManager.runLoop();
}
