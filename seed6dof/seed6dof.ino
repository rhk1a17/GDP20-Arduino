#include "BMI088.h"
 
float ax = 0, ay = 0, az = 0;
float gx = 0, gy = 0, gz = 0;
float mapped_ax;
int16_t temp = 0;
 
void setup(void)
{
    Wire.begin();
    Serial.begin(115200);
 
    while(!Serial);
    Serial.println("BMI088 Raw Data");
 
    while(1)
    {
        if(bmi088.isConnection())
        {
            bmi088.initialize();
            Serial.println("BMI088 is connected");
            break;
        }
        else Serial.println("BMI088 is not connected");
 
        delay(2000);
    }
}
 
void loop(void)
{    
    bmi088.getAcceleration(&ax, &ay, &az);
    bmi088.getGyroscope(&gx, &gy, &gz);
    temp = bmi088.getTemperature();
 /*
    // MAPPING
    mapped_ax = map(ax,-1000,1000,-90.0,90.0);
    Serial.print(mapped_ax);
    Serial.print(" deg");
    Serial.println();
*/

    Serial.print(ax);
    Serial.print(",");
    Serial.print(ay);
    Serial.print(",");
    Serial.print(az);
    Serial.print(",");
 /*
    Serial.print(gx);
    Serial.print(",");
    Serial.print(gy);
    Serial.print(",");

    Serial.print(gz);
    Serial.print(",");
 */
    //Serial.print(temp);
 
    Serial.println();
 
    delay(1000);
}
