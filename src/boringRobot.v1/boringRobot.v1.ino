#include <IoAbstraction.h>
#include "robot.h"

IoAbstractionRef arduinoIo = ioUsingArduino();

char slotString[20] = { 0 };

int taskId = -1;

Robot robot;
static const char *name = "boringRobot-v1.0";

void setup() {
  Serial.begin(115200);
  while(!Serial);

  log("Starting boringRobot");

  pinMode(ENCODERA_PIN, INPUT_PULLUP);

  taskManager.scheduleOnce(2000, twoSecondsStartUp);

  taskId = taskManager.scheduleFixedRate(1, oneSecondPulse, TIME_SECONDS);

  log("Waiting 32 milli second with yield in setup");
  taskManager.yieldForMicros(32000);
  log("Waited 32 milli second with yield in setup");

  taskManager.scheduleOnce(30000, [] {
    log("30 seconds up, stopping 1 second job");

    taskManager.cancelTask(taskId);
  });

  taskManager.scheduleFixedRate(5, [] { log(taskManager.checkAvailableSlots(slotString)); }, TIME_SECONDS);

  taskManager.scheduleFixedRate(100, onMicrosJob, TIME_MICROS);

  taskManager.setInterruptCallback (onInterrupt);
  taskManager.addInterrupt(arduinoIo, ENCODERA_PIN, CHANGE);
}

void onInterrupt(uint8_t bits) {
  log("Interrupt triggered");
  Serial.print("  Interrupt was ");
  Serial.println(bits);
}

int microCount = 0;

void onMicrosJob() {
  microCount++;
}

void oneSecondPulse() {
  log("One second pulse, microCount=", microCount);
}

void twoSecondsStartUp() {
  log("Two seconds start up");

  robot.init("boringRobot-v1.0");

  log(taskManager.checkAvailableSlots(slotString));
  if(taskManager.scheduleOnce(10000, twentySecondsUp) == 0xff) {
    log("Failed to register twenty second task");
  }
}

void twentySecondsUp() {
  log("Twenty seconds up");
}

void loop() {
  taskManager.runLoop();
}
