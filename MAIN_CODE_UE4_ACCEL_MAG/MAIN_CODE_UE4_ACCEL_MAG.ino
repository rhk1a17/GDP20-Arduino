//======================================Accel===================================
#include "BMI088.h"
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mapped_ax;
int16_t temp = 0;
//======================================Accel===================================

//======================================magneto===================================
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1();

#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

#define DECLINATION 0.0833 
// declination (in degrees) in Southampton

float mx, my, mz, heading;
//======================================magneto===================================

String long_str,rotation_val;
long rpmval;

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

void setup(void) {
  Serial.begin(115200);
  Wire.begin();
  Serial.setTimeout(5);
  
  //=======================Accel==========================
  while(1)
  {
    if(bmi088.isConnection())
    {
        bmi088.initialize();
        break;
    }
    delay(2000);
  }
  //=======================Accel==========================

  //=======================Megneto==========================
  // Try to initialise and warn if we couldn't detect the chip
  if (!lsm.begin())
  {
    Serial.println("No Magnetometer");
    while (1);
  }
  // helper to just set the default scaling we want, see above!
  setupSensor();
  //=======================Magneto==========================

  //=====================RPM Setup========================
  pinMode(RevSensePin, INPUT);
  attachInterrupt(digitalPinToInterrupt(RevSensePin), RevSenseISR, RISING);
  //=====================RPM Setup========================
}

void loop() {

    //======================================Accel===================================
    bmi088.getAcceleration(&ax, &ay, &az);
    mapped_ax = map(ax,-1000,1000,-90.0,90.0);
    //======================================Accel===================================
 
    //=====================================Magneto==================================
    lsm.read();  /* ask it to read in the data */ 
    /* Get a new sensor event */ 
    sensors_event_t a, m, g, temp;

    lsm.getEvent(&a, &m, &g, &temp);

    mx = m.magnetic.x;
    my = m.magnetic.y;
    mz = m.magnetic.z;

    //calculate heading

    if (my == 0) {
      heading = (mx < 0) ? PI : 0;
    }
    else {
      heading = atan2(mx, my);
    }

    
    //correct heading according to declination

    heading -= DECLINATION * PI / 180;
    if (heading > PI) {
      heading -= (2 * PI);
    }
    else if (heading < -PI) {
      heading += (2 * PI);
    }
    else if (heading < 0) {
      heading += 2 * PI;
    }

    
    //convert values in degree

    heading *= 180.0 / PI;
    //=====================================Magneto==================================

    // divide total val * weightage (0.9 steer 0.1 tilt) 
    // -0.45 and -0.05 for offset in UE4
    //rotation_val = String((((steerval/1023.0)*0.9)-0.45)+(((tiltval/1023.0)*0.1)-0.05)); 

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
        // tilt accel val + steer magneto val + rpm separated by comma
        long_str = String(mapped_ax) + "," + String(heading) + "," + rpmval; 
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

//===================================Magnetometer Setup=================
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);
}
//===================================Magnetometer Setup=================