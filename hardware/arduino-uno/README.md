
# Arduino UNO

Вместо головы у робота [Arduino Uno].

[Arduino Uno]: https://store.arduino.cc/usa/arduino-uno-rev3

The ATmega328 has 32 KB (with 0.5 KB occupied by the bootloader). It also has 2
KB of SRAM and 1 KB of EEPROM.

The board can operate on an external supply from 6 to 20 volts. If supplied
with less than 7V, however, the 5V pin may supply less than five volts and the
board may become unstable. If using more than 12V, the voltage regulator may
overheat and damage the board. The recommended range is 7 to 12 volts.


## PWM и таймеры

https://playground.arduino.cc/Code/PwmFrequency

PWM frequencies are tied together in pairs of pins. If one in a
pair is changed, the other is also changed to match:
* Pins 5 and 6 are paired on timer0
* Pins 9 and 10 are paired on timer1
* Pins 3 and 11 are paired on timer2

Note that this function will have side effects on anything else
that uses timers:
* Changes on pins 3, 5, 6, or 11 may cause the delay() and
  millis() functions to stop working. Other timing-related
  functions may also be affected.
* Changes on pins 9 or 10 will cause the Servo library to function
  incorrectly.


### Pins 5 and 6

```
Setting Divisor Frequency
0x01    1       62500
0x02    8        7812.5
0x03    64        976.5625
0x04    256       244.140625
0x05    1024       61.03515625


TCCR0B = TCCR0B & 0b11111000 | <setting>;
```

### Pins 9 and 10

```
Setting Divisor Frequency
0x01    1       31250
0x02    8        3906.25
0x03    64        488.28125
0x04    256       122.0703125
0x05    1024       30.517578125


TCCR1B = TCCR1B & 0b11111000 | <setting>;
```

### Pins 11 and 3

```
Setting Divisor Frequency
0x01    1       31250
0x02    8        3906.25
0x03    32        976.5625
0x04    64        488.28125
0x05    128       244.140625
0x06    256       122.0703125
0x07    1024       30.517578125


TCCR2B = TCCR2B & 0b11111000 | <setting>;
```

All frequencies are in Hz and assume a 16000000 Hz system clock.
