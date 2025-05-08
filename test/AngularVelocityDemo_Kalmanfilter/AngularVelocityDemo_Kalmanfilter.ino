
 
#include <Wire.h> 
#include "DFRobot_BNO055.h"

DFRobot_BNO055 mpu;
// Kalman Filter Variables
float Q_angle = 0.001;  // Process noise variance for the accelerometer
float Q_bias = 0.003;   // Process noise variance for the gyro bias
float R_measure = 0.03; // Measurement noise variance - adjust to see how much you trust the sensor

float angle = 0.0;      // The angle calculated by the Kalman filter
float bias = 0.0;       // The gyro bias calculated by the Kalman filter
float rate = 0.0;       // Unbiased rate calculated from the gyro and used by the filter

unsigned long lastTime = 0;


void setup() 
{
   Serial.begin(115200);
   while (!mpu.init())
   {
     Serial.println("ERROR! Unable to initialize the chip!");
     delay(50);
   }
//   mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);
   delay(100);
   Serial.println("Read Angular Velocity...");

}

void loop() 
{
  mpu.readAngularVelocity();  /* read Angular Velocity */
  
  //Serial.print("X: "); 
  Serial.println(mpu.GyrData.x, 3); 
  //Serial.print(" dps  "); 
  
  // Serial.print("Y: "); 
  // Serial.print(mpu.GyrData.y, 3); 
  // Serial.print(" dps ");
  
  // Serial.print("Z: "); 
  // Serial.print(mpu.GyrData.z, 3); 
  // Serial.println(" dps ");
  
  

  delay(200);
}

