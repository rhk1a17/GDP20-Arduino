//======================================PIN & VAR SETUP=================================
int trigPin2 = 11;    // Trigger pin for yDist2
int echoPin2 = 12;    // Echo pin for yDist2
long duration2, yDist2;

void setup() {
  //Serial Port begin
  Serial.begin (9600);
  Serial.setTimeout(5);

  //Define inputs and outputs
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
}
 
void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  pinMode(echoPin2, INPUT);
  duration2 = pulseIn(echoPin2, HIGH);
 
  // Convert the time into a distance
  yDist2 = (duration2/2) / 29.1;

  //======================================UE4=====================================
  if (!Serial.available()) return;

  String str = Serial.readString();

  if (str == "ultrasonic")
  {
    long int lVal = yDist2;
    byte lBuffer[] = {
      byte(lVal & 0xff),
      byte(lVal >> 8 & 0xff),
      byte(lVal >> 16 & 0xff),
      byte(lVal >> 24 & 0xff)
    };
    Serial.write(lBuffer, 4);
  }
  
  //======================================UE4=====================================
}