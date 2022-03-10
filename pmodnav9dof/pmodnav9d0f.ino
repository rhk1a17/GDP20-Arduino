/************************************************************************

  Test of Pmod NAV (Based on Jim Lindblom's program)

*************************************************************************

  Description: Pmod_NAV
  All data (accelerometer, gyroscope, magnetometer) are displayed
  In the serial monitor

  Material
  1. Arduino Uno
  2. Pmod NAV (dowload library
  https://github.com/sparkfun/SparkFun_LSM9DS1_Arduino_Library )
  Licence Beerware

  Wiring
  Module<--------------------> Arduino
  J1 pin 6 (3V3)     to        3V3
  J1 pin 5 (GND)     to        GND
  J1 pin 4 (SCK)     to        A5 (SCL)
  J1 pin 2 (SDI)     to        A4 (SDA)

************************************************************************/


// The earth's magnetic field varies according to its location.

// Add or subtract a constant to get the right value

// of the magnetic field using the following site

// http://www.ngdc.noaa.gov/geomag-web/#declination


#define DECLINATION 0.0833
// declination (in degrees) in Southampton UK.


/************************************************************************/


#define PRINT_CALCULATED  
//print calculated values

//#define PRINT_RAW       //print raw data



// Call of libraries

#include <Wire.h>
#include <SparkFunLSM9DS1.h>

// defining module addresses

#define LSM9DS1_M 0x1E  
//magnetometer

#define LSM9DS1_AG 0x6B 
//accelerometer and gyroscope


LSM9DS1 imu; 
// Creation of the object


void setup(void)
{
  Serial.begin(115200); 
// initialization of serial communication

  Wire.begin();     
//initialization of the I2C communication

  imu.settings.device.commInterface = IMU_MODE_I2C; 
// initialization of the module

  imu.settings.device.mAddress = LSM9DS1_M;        
//setting up addresses

  imu.settings.device.agAddress = LSM9DS1_AG;
  if (!imu.begin()) 
//display error message if that's the case

  {
    Serial.println("Communication problem.");
    while (1);
  }
}

void loop()
{
  
//measure

  if ( imu.gyroAvailable() )
  {
    imu.readGyro(); 
//measure with the gyroscope

  }
  if ( imu.accelAvailable() )
  {
    imu.readAccel(); 
//measure with the accelerometer

  }
  if ( imu.magAvailable() )
  {
    imu.readMag(); 
//measure with the magnetometer

  }

  
//display data

  //printGyro(); 
// Print "G: gx, gy, gz"

  //printAccel(); 
// Print "A: ax, ay, az"

  //printMag(); 
// Print "M: mx, my, mz"

  printAttitude(imu.ax, imu.ay, imu.az, -imu.my, -imu.mx, imu.mz); 
//print pitch, roll and heading

  //Serial.println();
  delay(1000);
}

//display gyroscope data

void printGyro()
{
  Serial.print("G: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcGyro(imu.gx), 2);  
//calculate angular velocity

  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gy), 2);
  Serial.print(", ");
  Serial.print(imu.calcGyro(imu.gz), 2);
  Serial.println(" deg/s");               
//measured in deg/s

#elif defined PRINT_RAW
  Serial.print(imu.gx);                   
//or display raw data

  Serial.print(", ");
  Serial.print(imu.gy);
  Serial.print(", ");
  Serial.println(imu.gz);
#endif
}

//display accelerometer data

void printAccel()
{
  Serial.print("A: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcAccel(imu.ax), 2);   
//calculate acceleration

  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.ay), 2);
  Serial.print(", ");
  Serial.print(imu.calcAccel(imu.az), 2);
  Serial.println(" g");                    
//measured in g

#elif defined PRINT_RAW
  Serial.print(imu.ax);                   
//or display raw data

  Serial.print(", ");
  Serial.print(imu.ay);
  Serial.print(", ");
  Serial.println(imu.az);
#endif
}

//display magnetometer data

void printMag()
{
  Serial.print("M: ");
#ifdef PRINT_CALCULATED
  Serial.print(imu.calcMag(imu.mx), 2);   
//calculate magnetic field components

  Serial.print(", ");
  Serial.print(imu.calcMag(imu.my), 2);
  Serial.print(", ");
  Serial.print(imu.calcMag(imu.mz), 2);
  Serial.println(" gauss");                    
//measured in gauss

#elif defined PRINT_RAW
  Serial.print(imu.mx);                   
//or display raw data

  Serial.print(", ");
  Serial.print(imu.my);
  Serial.print(", ");
  Serial.println(imu.mz);
#endif
}

//display additional calculated values

void printAttitude(float ax, float ay, float az, float mx, float my, float mz)
{
  float roll = atan2(ay, az); 
//calculate roll

  float pitch = atan2(-ax, sqrt(ay * ay + az * az));  
//calculate pitch

  float heading;  
//variable for hading


  
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
  pitch *= 180.0 / PI;
  roll *= 180.0 / PI;

  
//display calculated data
/*
  Serial.print("Pitch, Roll: ");
  Serial.print(pitch, 2);
  Serial.print(", ");
  Serial.print(roll, 2);
  Serial.println(" °");
  */
  Serial.print("Heading: ");
  Serial.print(heading, 2);
  Serial.println(" °");
}