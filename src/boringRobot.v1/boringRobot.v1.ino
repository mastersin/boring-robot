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

int fr = 0;
int fl = 0;
int br = 0;
int bl = 0;
int r = 0;
int l = 0;

int lp = 0;
int rp = 0;

static int BASE_POWER = 15;
void easy_go()
{
  fr = robot.analogSensor(ForwardSensorRight);
  fl = robot.analogSensor(ForwardSensorLeft);
  br = robot.analogSensor(BackwardSensorRight);
  bl = robot.analogSensor(BackwardSensorLeft);
  r = robot.analogSensor(CentralSensorRight);
  l = robot.analogSensor(CentralSensorLeft);

  static int I = 0;
  int er = l - r;
  int k1 = 1;
  int k2 = 20;
  int fs = fl + fr;
  int acc = 0;
  if (fs < 1000)
    k2 = 30;
  if (fs > 800 && br < 500 && bl < 500)
    acc = 10;

  I += er;
  if (I > 10000)
    I = 10000;
  if (I > -10000)
    I = -10000;

//  if ((br > 400 || bl > 400) && fs > 1000) {
//    k2 = 15;
//  }
  int adj = er * k1 / k2;

  lp = BASE_POWER + acc + (adj > 0 ? adj * 2 / 3 : adj);// + I / 1000;
  rp = BASE_POWER + acc - (adj < 0 ? adj * 2 / 3 : adj);// - I / 1000;

  if (lp < 0) lp = 0;
  if (rp < 0) rp = 0;
  if (lp > 60) lp = 65;
  if (rp > 60) rp = 65;

//  if (br > 500) {
//    lp -= 20;
//    rp += 20;
//  }
//  if (bl > 500) {
//    rp -= 20;
//    lp += 20;
//  }
}

static int state = 0;
static bool programStarted = false;
void program()
{
  if (!programStarted) {
    robot.setPowerA(0);
    robot.setPowerB(0);
    return;
  }

  easy_go();

  switch(state) {
    case 0:
      if (br > 500 || bl > 500)
        state = 1;
      break;
    case 1:
      rp = 0;
      lp = 40;
      robot.resetEncoders();
      state = 2;
    case 2:
      if (robot.getEncoderA() > 15)
        state = 0;
      break;
  }

  robot.setPowerA(lp);
  robot.setPowerB(rp);
}

void startProgram()
{
  if (programTask < 0) {
    programTask = taskManager.scheduleFixedRate(10, program);
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
  log("Pulse, state = ", state);

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
