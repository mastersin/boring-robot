
#include <Arduino.h>

static int counter = 0;

void led_enable(bool);

void setup () {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop () {
    led_enable(counter & 1);
    Serial.println(counter++);
    delay(500);
}
