#ifndef IMU_H
#define IMU_H
#include <Wire.h>
#include <DFRobot_BNO055.h>
#include <Arduino.h>
class Imu {
  private:
    DFRobot_BNO055 mpu;
  public:
    
    void init() {
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
    float angularVelocityXaxis() {
      mpu.readAngularVelocity();
      return mpu.GyrData.x;
    }
    float angularVelocityYaxis() {
      mpu.readAngularVelocity();
      return mpu.GyrData.y;
    }
    float angularVelocityZaxis() {
      mpu.readAngularVelocity();
      return mpu.GyrData.z;
    }
    float eulerAngleX() {
      mpu.readEuler();
      return mpu.EulerAngles.x;
    }
    float eulerAngleY() {
      mpu.readEuler();
      return mpu.EulerAngles.y;
    }
    float eulerAngleZ() {
      mpu.readEuler();
      return mpu.EulerAngles.z;
    }
};
#endif