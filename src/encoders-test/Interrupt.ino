const byte ledPin = 13;
const byte interruptPin1 = 18;
const byte interruptPin2 = 19;
volatile byte state = LOW;

void setup() {
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin1, INPUT_PULLUP);
  pinMode(interruptPin2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin1), encoderISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(interruptPin2), encoderISR2, CHANGE);
  Serial.begin(115200);
}

volatile unsigned long counter1 = 0;
unsigned long last_counter1 = counter1;
unsigned long diff_counter1 = counter1;
volatile unsigned long counter2 = 0;
unsigned long last_counter2 = counter2;
unsigned long diff_counter2 = counter2;

unsigned long velocity1 = 0;
unsigned long velocity2 = 0;
unsigned long last_millis = 0;

void loop() {
  unsigned long new_millis = millis();

  if (counter1 != last_counter1 || counter2 != last_counter2) {
    last_counter1 = counter1;
    last_counter2 = counter2;

    Serial.print(counter1);
    Serial.print(" - ");
    Serial.print(velocity1);
    Serial.print(" : ");
    Serial.print(counter2);
    Serial.print(" - ");
    Serial.println(velocity2);
    
    state = !state;
    digitalWrite(ledPin, state);
  }

  if (new_millis - last_millis > 100)
  {
    unsigned long diff1 = last_counter1 - diff_counter1;
    unsigned long diff2 = last_counter2 - diff_counter2;

    velocity1 = diff1;
    velocity2 = diff2;
    
    last_millis = new_millis;
    diff_counter1 = last_counter1;
    diff_counter2 = last_counter2;
  }
}

void encoderISR1() {
  counter1++;
}

void encoderISR2() {
  counter2++;
}
