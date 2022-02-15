int blackpotPin = 0;    // select the input pin for the potentiometer
int blackval = 0;       // variable to store the value coming from the sensor
int bluepotPin = 1;    
int blueval = 0;       

void setup() {
    Serial.begin(9600);
}

void loop() {
    blackval = analogRead(blackpotPin);    // read the value from the sensor
    Serial.println(blackval);
    blueval = analogRead(bluepotPin);    // read the value from the sensor
    Serial.println(blueval);
    delay(10);
}