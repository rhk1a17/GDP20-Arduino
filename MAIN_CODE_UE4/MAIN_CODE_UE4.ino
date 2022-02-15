int tiltpotPin = 0;    //tilt pin ANALOG 0  
int steerpotPin = 1;     //steering pin ANALOG 1
String long_str,rotation_val;
long steerval, tiltval, rpmval;

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(5);
}

void loop() {
    tiltval = analogRead(tiltpotPin);    // TILT
    steerval = analogRead(steerpotPin);    // STEER

    // divide total val * weightage (0.9 steer 0.1 tilt) 
    // -0.45 and -0.05 for offset in UE4
    rotation_val = String((((steerval/1023.0)*0.9)-0.45)+(((tiltval/1023.0)*0.1)-0.05)); 

    //======================================RPM=====================================
    rpmval = 0;
    //======================================RPM=====================================
    
    //======================================UE4=====================================
    if (!Serial.available()) return;

    String str = Serial.readString();

    if (str == "data")
    {
        long_str = rotation_val + "," + rpmval; // TILT + STEER separated by comma
        Serial.println(long_str);
    }

    //======================================UE4=====================================
}