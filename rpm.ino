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

void setup()
{
  Serial.begin(9600);
  pinMode(RevSensePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RevSensePin), RevSenseISR, RISING);
}

void RevSenseISR()
{
  static unsigned long revSensePreviousMicros = 0;  // 'static' to retain value between calls

  RevSenseTimeMicros = micros();
  RevTimeMicros = RevSenseTimeMicros - revSensePreviousMicros; // Time for last revolution
  revSensePreviousMicros = RevSenseTimeMicros;
}

void loop()
{
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
      displayRPM(newRPM);
  }
}

void displayRPM(unsigned RPM)
{
  float WheelRPM = RPM/GearRatio;
  float metersPerMinute = WheelRPM * WheelRadiusInMeters;
  ;
 

  Serial.print("WheelRPM = "); //print the word "RPM".
  Serial.print(WheelRPM); // print the rpm value.
  Serial.print("\t\t Linear Speed = ");
  Serial.print(metersPerMinute/60); //print the linear velocity value.
  Serial.println(" m/s");
}
