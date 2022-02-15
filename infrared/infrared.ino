int val; // analog reading from IR sensors
float distance; // distace float
float distance_1;
int sum = 0;

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
    for (int i = 0; i <= 10; i++) {
    val = analogRead(0); // read the value from the IR sensor
    distance = 12343.85 * pow(val, -1.15);    // distance in cm; for accurate sensor measurements, calibration is required
    sum = sum + distance;
    delay(10);   // sampling interval
    }

  distance_1 = sum/10; // averaging over 10 readings
  Serial.println(distance_1);
  sum = 0; //clear sum
}