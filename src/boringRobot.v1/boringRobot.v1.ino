#include <IoAbstraction.h>
#include "robot.h"

IoAbstractionRef arduinoIo = ioUsingArduino();

char slotString[20] = { 0 };

int pulseTask = -1;
int pollTask = -1;
int programTask = -1;

Robot robot;
static const char *name = "boringRobot-v1.0";

void setup() {
  Serial.begin(115200);
  while(!Serial);

  log("Setup boringRobot");

  taskManager.scheduleOnce(1000, oneSecondsStartUp);
  pulseTask = taskManager.scheduleFixedRate(1, oneSecondPulse, TIME_SECONDS);

  taskManager.scheduleFixedRate(5, [] { log(taskManager.checkAvailableSlots(slotString)); }, TIME_SECONDS);
}

static bool programStarted = false;
void program()
{
    if (!programStarted)
      return;

    robot.setPowerA(100);
    robot.setPowerB(100);
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
  robot.showPoll();
}

void oneSecondsStartUp() {
  log("Initial boringRobot");

  robot.init("boringRobot-v1.0");
  robot.setButtonCallbacks(startStopProgram, startProgram, nextInfo);
  robot.setStatus("Ready");

  taskManager.scheduleFixedRate(100, [] {
    robot.poll();
  });
}

void loop() {
  taskManager.runLoop();
}
