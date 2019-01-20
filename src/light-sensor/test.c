
const int sensorPin = 0; //analog pin 0

void setup(){
    Serial.begin(9600);
}

void loop(){
    int val = analogRead(sensorPin);
    Serial.println(val);
    delay(100);
}
