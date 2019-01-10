
#include <Arduino.h>

void led_enable(bool enable) {
    digitalWrite(LED_BUILTIN, enable? HIGH: LOW);
}
