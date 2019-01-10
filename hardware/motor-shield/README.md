
# Motor Shield

Вместе с платформой мне достался Motor Shield. Очень похоже на
синенький клон [легенды], только микрухи не впаяны.

[легенды]: https://learn.adafruit.com/adafruit-motor-shield?view=all


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
