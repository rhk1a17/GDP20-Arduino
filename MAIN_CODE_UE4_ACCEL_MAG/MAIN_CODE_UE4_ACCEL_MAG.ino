//======================================Constants===================================
// Steer and tilt weightage to tune responsiveness of each movement
// increasing values will reflect directly on UE4
float steer_weightage = 0.9;
float tilt_weightage = 0.5;
// global variable setup 
String long_str,rotation_val;
long rpmval;
//======================================Constants===================================

//========================================Accel=====================================
// library and constants setup for accelerometer
#include "BMI088.h"
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mapped_ax;
int16_t temp = 0;
//========================================Accel=====================================

//=======================================magneto====================================
// library and constant setup for magnetomter
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>  // not used in this demo but required!
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(); // magnetometer object declaration

//magnetometer i2c addresses declaration
#define LSM9DS1_SCK A5
#define LSM9DS1_MISO 12
#define LSM9DS1_MOSI A4
#define LSM9DS1_XGCS 6
#define LSM9DS1_MCS 5

#define DECLINATION 0.0833 
// declination (in degrees) in Southampton

float mx, my, mz, heading, bench_heading, mapped_heading;
//======================================magneto===================================

//===========================RPM & fan constants==================================
const byte RevSensePin = 2; // digital pin 2 on arduino setup. (change according to connection)
const float WheelRadiusInMeters = 0.33; //Wheel radius
const float SFWRadiusInMeteres = 0.05; // Sub flywheel radius
const unsigned long DisplayIntervalMillis = 1;  // Update once per milliseconds
const unsigned long MaxRevTimeMicros = 500000UL; // >0.5 seconds per revolution counts as 0 RPM (reduce for faster stop reaction)

// Variables used in the ISR and in the main code must be 'volatile'
volatile unsigned long RevSenseTimeMicros = 0;  //  Time that the rising edge was sensed
volatile unsigned long RevTimeMicros = 0;  // Microseconds between consecutive pulses

// Useful constants:
const unsigned long SixtySecondsInMicros = 60000000UL;
const float WheelCircumferenceInMeters = TWO_PI * WheelRadiusInMeters;
const float GearRatio = WheelRadiusInMeters/SFWRadiusInMeteres;
const float SFWCircuferenceInMeters = TWO_PI * SFWRadiusInMeteres;

int max_rpm=300; // fan max rpm
int fan_pin=9; // fan pwm pin
float value; // constant
//===========================RPM & fan constants==================================

void setup(void) {
  Serial.begin(115200); // baud rate (must be matched to UE4)
  Wire.begin();
  Serial.setTimeout(5);
  //=================================Accel====================================
  while(1)
  {
    // setup connection with accelerometer
    if(bmi088.isConnection())
    {
      bmi088.initialize();
      break;
    }
    delay(100);
  }
  //=================================Accel====================================
  //================================Megneto===================================
  // Try to initialise and warn if we couldn't detect the magnetometer
  if (!lsm.begin())
  {
    Serial.println("No Magnetometer");
    while (1);
  }
  // helper to just set the default scaling we want, see declaration at end of code
  // includes hard offset and obtaining datum at each startup
  setupSensor();
  //================================Megneto===================================
  //============================RPM & fan Setup===============================
  pinMode(RevSensePin, INPUT); //RPM pin setup
  attachInterrupt(digitalPinToInterrupt(RevSensePin), RevSenseISR, RISING); //setup interrupt function for RPM

  pinMode(fan_pin,OUTPUT); // fan pin setup
  analogWrite(fan_pin,0); // set fan to 0
  //============================RPM & fan Setup===============================
}

void loop() {
    //======================================Accel===================================
    bmi088.getAcceleration(&ax, &ay, &az); // get acceleration value, ranged from -1000 to +1000 in all 3 axes
    mapped_ax = map(ax,-1000,1000,-90.0,90.0); // mapping acceleration in x to +90 and -90 degrees
    //======================================Accel===================================
 
    //=====================================Magneto==================================
    lsm.read();  // read in IMU data, including all 3 acceleromter, magnetometer, and gyro data
    sensors_event_t a, m, g, temp; // call event to obtain data

    lsm.getEvent(&a, &m, &g, &temp); //get data

    // raw magnetic field in all 3 axes
    mx = m.magnetic.x; 
    my = m.magnetic.y;
    mz = m.magnetic.z;

    // calibration, offset (hard offset calibrated)
    mx = mx - (-0.83);
    my = my - (23.61);
    mz = mz - (-10.35);

    //calculate current heading
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
    heading = (heading - bench_heading)*-1; // calculate heading relative to datum
    //mapping heading from +90 and -90 degree to +-100, with basic filter
    mapped_heading = map(heading*100,-9000,9000,-100,100); 
    //scaling calibrated heading to -100 to 100 with -90deg to 90deg
    //=====================================Magneto==================================

    //====================================RPM & fan=================================
    static unsigned previousRPM; //previous rpm to check if its the same

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

    value=map(rpmval,0,max_rpm,0,255); // fan speed value
    analogWrite(fan_pin,value); // write fan speed value
    //====================================RPM & fan=================================
    
    //======================================Rotation Val=====================================
    // rotation value sent to UE4
    rotation_val = String(((mapped_heading/90.0)*steer_weightage)+(((mapped_ax/90.0)*tilt_weightage))); 
    //======================================Rotation Val=====================================

    //===========================================UE4=========================================
    if (!Serial.available()) return; //check if UE4 is connected

    String str = Serial.readString(); // read string from UE4 block
    
    if (str == "data") // string read from UE4 must be identical to this
    {
        // tilt accel val + steer magneto val + rpm separated by comma
        // any additional data can be added to this line to be sent to UE4 with comma (must be a string)
        long_str = String(rotation_val) + "," + rpmval; //concatenate
        Serial.println(long_str); // write to UE4
    }
    Serial.flush(); // flush port
    //===========================================UE4=========================================
}

//=====================================RPM Interrupt function================================
void RevSenseISR()
{
  static unsigned long revSensePreviousMicros = 0;  // 'static' to retain value between calls

  RevSenseTimeMicros = micros();
  RevTimeMicros = RevSenseTimeMicros - revSensePreviousMicros; // Time for last revolution
  revSensePreviousMicros = RevSenseTimeMicros;
}
//=====================================RPM Interrupt function================================

//========================================Magnetometer Setup=================================
void setupSensor()
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  lsm.read();  //same as above
  sensors_event_t a, m, g, temp;

  lsm.getEvent(&a, &m, &g, &temp);

  mx = m.magnetic.x;
  my = m.magnetic.y;
  mz = m.magnetic.z;

  //calibrate magnetometer, hard offset
  mx = mx - (-0.83);
  my = my - (23.61);
  mz = mz - (-10.35);

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
  bench_heading = heading; // setup datum heading
}
//===================================Magnetometer Setup==============================