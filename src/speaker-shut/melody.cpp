/*
  Melody

  http://www.arduino.cc/en/Tutorial/Tone
*/

#include <Arduino.h>
#include "pitches.h"


struct Note {
    int duration;
    int pitch;
};

#define NOTE_0 0
#define NOTE(dur, pitch) \
{ 1536 / dur, NOTE_ ## pitch }

static const Note melody[] = {
    NOTE( 8, A4 ),
    NOTE( 8, 0  ),
    NOTE( 8, E5 ),
    NOTE( 8, A4 ),

    NOTE( 8, F5 ),
    NOTE( 8, 0  ),
    NOTE( 8, E5 ),
    NOTE( 8, D5 ),

    NOTE( 8, E5 ),
    NOTE( 8, 0  ),
    NOTE( 8, D5 ),
    NOTE( 8, E5 ),

    NOTE( 8, F5 ),
    NOTE( 8, F5 ),
    NOTE( 8, E5 ),
    NOTE( 8, D5 ),

    NOTE( 8, A4 ),
    NOTE( 8, 0  ),
    NOTE( 8, E5 ),
    NOTE( 8, A4 ),

    NOTE( 8, F5 ),
    NOTE( 8, 0  ),
    NOTE( 8, E5 ),
    NOTE( 8, D5 ),

    NOTE( 8, C5 ),
    NOTE( 8, 0  ),
    NOTE( 8, D5 ),
    NOTE( 8, C5 ),

    NOTE( 8, B4 ),
    NOTE( 8, B4 ),
    NOTE( 8, C5 ),
    NOTE( 8, B4 ),
};

static const Note* melody_end = melody + (sizeof(melody) / sizeof(melody[0]));
#define SPEAKER 8


void play() {
    for (Note const* note = melody; note != melody_end; ++note) {
        tone(SPEAKER, note->pitch, note->duration);
        if (note->pitch) digitalWrite(LED_BUILTIN, HIGH);
        delay(note->duration >> 2);
        digitalWrite(LED_BUILTIN, LOW);
        delay(note->duration);
    }
    noTone(SPEAKER);
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    play();
}
