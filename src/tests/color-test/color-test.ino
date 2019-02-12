#define COLOR_LED_PIN  28
#define COLOR_S0_PIN   29
#define COLOR_S1_PIN   30
#define COLOR_S2_PIN   31
#define COLOR_S3_PIN   32
#define COLOR_OUT_PIN  2

const int s0 = COLOR_S0_PIN;
const int s1 = COLOR_S1_PIN;
const int s2 = COLOR_S2_PIN;
const int s3 = COLOR_S3_PIN;
const int out = COLOR_OUT_PIN;
const int led = COLOR_LED_PIN;

// values
int red = 0;
int green = 0;
int blue = 0;

void interruptHandlerColorCounter();
#define TCS3200_PULSES_INTERVAL 100
void setup() {
    Serial.begin(9600);

    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(led, OUTPUT);
    pinMode(out, INPUT_PULLUP);

    digitalWrite(s0, HIGH);
    digitalWrite(s1, HIGH);
    digitalWrite(led, LOW);
    attachInterrupt(digitalPinToInterrupt(COLOR_OUT_PIN), interruptHandlerColorCounter, CHANGE);
}

void loop() {
    color();
    Serial.print("RED ");
    Serial.print(red, DEC);
    Serial.print(" GREEN ");
    Serial.print(green, DEC);
    Serial.print(" BLUE ");
    Serial.print(blue, DEC);

    if (red < blue && red < green && red < 20) {
        if (red <= 10 && green <= 10 && blue <= 10) {
            Serial.println("  WHILE");
        } else {
            Serial.println(" - (Red Color)");
        }
    } else if (blue < red && blue < green) {
        if (red <= 10 && green <= 10 && blue <= 10) {
            Serial.println("  WHILE");
        } else {
            Serial.println(" - (Blue Color)");
        }
    } else if (green < red && green < blue) {
        if (red <= 10 && green <= 10 && blue <= 10) {
            Serial.println("  WHILE");
        } else {
            Serial.println(" - (Green Color)");
        }
    } else {
        Serial.println("  IDK");
    }
    delay(1000);
}

volatile unsigned long counter;


void color() {
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    //red = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    counter = 0;
    delay(TCS3200_PULSES_INTERVAL);
    red = counter;
    digitalWrite(s3, HIGH);
    counter = 0;
    delay(TCS3200_PULSES_INTERVAL);
    blue = counter;
    //blue = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
    digitalWrite(s2, HIGH);
    counter = 0;
    delay(TCS3200_PULSES_INTERVAL);
    green = counter;
    //green = pulseIn(out, digitalRead(out) == HIGH ? LOW : HIGH);
}

void interruptHandlerColorCounter()
{
  //Serial.println(counter);
  ++counter;
}
