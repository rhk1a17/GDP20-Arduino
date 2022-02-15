//======================================PIN & VAR SETUP=================================
int trigPin1 = 9;     // Trigger pin for yDist1
int echoPin1 = 10;    // Echo pin for yDist1
int trigPin2 = 11;    // Trigger pin for yDist2
int echoPin2 = 12;    // Echo pin for yDist2
long duration1, yDist1;
long duration2, yDist2;
long tilt_angle;

//======================================INPUT PARAMETER SETUP============================
long xDistance_cm = 100.00; //x distance between 2 ultrasonic sensors in cm


void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
}
 
void loop() {
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin1, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin1, INPUT);
  duration1 = pulseIn(echoPin1, HIGH);

  digitalWrite(trigPin2, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin2, LOW);

  pinMode(echoPin2, INPUT);
  duration2 = pulseIn(echoPin2, HIGH);
 
  // Convert the time into a distance
  yDist1 = (duration1/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  yDist2 = (duration2/2) / 29.1;

  tilt_angle = (atan((yDist1-yDist2)/xDistance_cm))*(180/PI);

  Serial.print(tilt_angle);
  Serial.print("deg");
  Serial.println();
  
  delay(250);
}