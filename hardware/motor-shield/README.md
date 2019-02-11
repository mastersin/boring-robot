
# Motor Shield

Вместе с платформой мне достался Motor Shield. Очень похоже на
синенький клон [легенды], только микрухи не впаяны.

[легенды]: https://learn.adafruit.com/adafruit-motor-shield?view=all

Родная либа: https://github.com/adafruit/Adafruit-Motor-Shield-library


For Arduino Mega: (tested on Arduino Mega 2560)
timer 0 (controls pin 13, 4)
timer 1 (controls pin 12, 11)
timer 2 (controls pin 10, 9)
timer 3 (controls pin 5, 3, 2)
timer 4 (controls pin 8, 7, 6)

https://playground.arduino.cc/Main/TimerPWMCheatsheet
How to adjust Arduino PWM frequencies

by macegr in this forum post http://forum.arduino.cc/index.php?topic=16612#msg121031
Pins 5 and 6: controlled by Timer 0 in fast PWM mode (cycle length = 256)

Setting 	Divisor 	Frequency
0x01 	 	1 	 	62500
0x02  		8 	 	7812.5
0x03  		64 	 	976.5625   <--DEFAULT
0x04 	 	256 	 	244.140625
0x05 	 	1024 	 	61.03515625

TCCR0B = (TCCR0B & 0b11111000) | <setting>;

Pins 9 and 10: controlled by timer 1 in phase-correct PWM mode (cycle length = 510)

Setting 	Divisor 	Frequency
0x01 	 	1 	 	31372.55
0x02 	 	8 	 	3921.16
0x03  		64 	 	490.20   <--DEFAULT
0x04  		256 	 	122.55
0x05 	 	1024 	 	30.64

TCCR1B = (TCCR1B & 0b11111000) | <setting>;

Pins 11 and 3: controlled by timer 2 in phase-correct PWM mode (cycle length = 510)

Setting 	Divisor 	Frequency
0x01 	 	1  		31372.55
0x02 	 	8 	 	3921.16
0x03  		32  		980.39
0x04 	 	64 	 	490.20   <--DEFAULT
0x05 	 	128  		245.10
0x06  		256  		122.55
0x07 	 	1024  		30.64

TCCR2B = (TCCR2B & 0b11111000) | <setting>;

All frequencies are in Hz and assume a 16000000 Hz system clock.


• ICP5 – Port L, Bit 1
ICP5, Input Capture Pin 5: The PL1 pin can serve as an Input Capture pin for Timer/Counter5.
• ICP4 – Port L, Bit 0
ICP4, Input Capture Pin 4: The PL0 pin can serve as an Input Capture pin for Timer/Counter4.





![Схема](mshieldv12schem.png)


## Подключение

Если его надеть на ардуинку, свободных пинов не останется (почти).
Можно, например, впаять гребёнку на A0-A5, но неохота и всё равно
мало.

Поэтому другой вариант:
* питание для движков делаем независимым
* шилд не надеваем а монтируем рядом (благо места полно)
* подключаем только нужные пины проводочками

Так как у нас только DC движки, подключить нужно:
* Digital pin 4, 7, 8 and 12 are used to drive the DC/Stepper motors
  via the 74HC595 serial-to-parallel latch
* Digital pin 11: DC Motor #1 / Stepper #1 (activation/speed control)
* Digital pin 3: DC Motor #2 / Stepper #1 (activation/speed control)
* Digital pin 5: DC Motor #3 / Stepper #2 (activation/speed control)
* Digital pin 6: DC Motor #4 / Stepper #2 (activation/speed control)

Причём speed control нужно подключать только те, что реально
используются (2 из 4).

Получаем 4 логических и 2 шимных пина на моторы.

Если подцепиться к выводам 74HC595, можно получить ещё 4
небыстрых логических выхода нахаляву.
