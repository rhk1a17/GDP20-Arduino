int tiltpotPin = 0;    //tilt pin ANALOG 0  
int steerpotPin = 1;     //steering pin ANALOG 1
String long_str,rotation_val;
long steerval, tiltval, rpmval;

//==========================RPM constants==================================
const byte RevSensePin = 2;
const float WheelRadiusInMeters = 0.33;
const float SFWRadiusInMeteres = 0.05;
const unsigned long DisplayIntervalMillis = 10;  // Update once per second
const unsigned long MaxRevTimeMicros = 2000000UL; // >2 seconds per revolution counts as 0 RPM

// Variables used in the ISR and in the main code must be 'volatile'
volatile unsigned long RevSenseTimeMicros = 0;  //  Time that the rising edge was sensed
volatile unsigned long RevTimeMicros = 0;  // Microseconds between consecutive pulses

// Useful constants:
const unsigned long SixtySecondsInMicros = 60000000UL;
const float WheelCircumferenceInMeters = TWO_PI * WheelRadiusInMeters;
const float GearRatio = WheelRadiusInMeters/SFWRadiusInMeteres;
const float SFWCircuferenceInMeters = TWO_PI * SFWRadiusInMeteres;
//==========================RPM constants==================================

void setup() {
    Serial.begin(9600);
    Serial.setTimeout(5);
    //=====================RPM Setup========================
    pinMode(RevSensePin, INPUT);
    attachInterrupt(digitalPinToInterrupt(RevSensePin), RevSenseISR, RISING);
    //=====================RPM Setup========================
}

void loop() {
    tiltval = analogRead(tiltpotPin);    // TILT
    steerval = analogRead(steerpotPin);    // STEER

    // divide total val * weightage (0.9 steer 0.1 tilt) 
    // -0.45 and -0.05 for offset in UE4
    rotation_val = String((((steerval/1023.0)*0.9)-0.45)+(((tiltval/1023.0)*0.1)-0.05)); 

    //======================================RPM=====================================
    static unsigned previousRPM;

    // Only update the display once per DisplayIntervalMillis
    unsigned long currentMillis = millis();
    static unsigned long previousMillis = 0;
    if (currentMillis - previousMillis >= DisplayIntervalMillis)
    {
        previousMillis += DisplayIntervalMillis;

        // With interrupts disabled, make local copies of volatile variables
        // This is so the ISR can't change them while we read them
        noInterrupts();
        unsigned long revSenseTimeMicros = RevSenseTimeMicros;  //  Time that the last rising edge was sensed
        unsigned long revTimeMicros = RevTimeMicros;  // Microseconds between consecutive pulses
        interrupts();

        // Calculate RPM
        unsigned newRPM;
        if (micros() - revSenseTimeMicros > MaxRevTimeMicros)
        newRPM = 0;   // Going so slow we're essentially stopped
        else
        newRPM = SixtySecondsInMicros / revTimeMicros;
        float WheelRPM = newRPM/GearRatio;
        float metersPerMinute = WheelRPM * WheelRadiusInMeters;
        rpmval = WheelRPM;
    }
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

//===================================RPM Interrupt function=================
void RevSenseISR()
{
  static unsigned long revSensePreviousMicros = 0;  // 'static' to retain value between calls

  RevSenseTimeMicros = micros();
  RevTimeMicros = RevSenseTimeMicros - revSensePreviousMicros; // Time for last revolution
  revSensePreviousMicros = RevSenseTimeMicros;
}
//===================================RPM Interrupt function=================