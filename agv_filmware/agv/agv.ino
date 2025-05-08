#include "imu.h"
#include "barrometer.h"
#include "robotControl.h"
#include "motor.h"

// Pin definitions
#define ENCODERA1 3
#define ENCODERA2 5
#define ENCODERB1 2
#define ENCODERB2 4
#define ENABLEA 9
#define ENABLEB 11
#define IN1 12  // Motor Kanan
#define IN2 13  // Motor Kanan
#define IN3 7   // Motor Kiri
#define IN4 8   // Motor Kiri

// PID tuning parameters
#define KP 1.2  // Proportional tuning
#define KI 0.0  // Integral tuning
#define KD 0.8  // Derivative tuning

// Create objects
Imu imu;
Barrometer bmp;
Motor rightMotor(ENABLEA, IN1, IN2, ENCODERA1, ENCODERA2, KP, KI, KD);
Motor leftMotor(ENABLEB, IN3, IN4, ENCODERB1, ENCODERB2, KP, KI, KD);
RobotControl agvControl(&rightMotor, &leftMotor, &imu, &bmp);

void setup() {
  // Initialize components
  Serial.begin(115200);
  imu.init();
  bmp.init();
  rightMotor.init();
  leftMotor.init();
  agvControl.init();
  
  // Set up encoder interrupts
  attachInterrupt(digitalPinToInterrupt(ENCODERA1), rightEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODERB1), leftEncoderISR, RISING);
}

void loop() {
  // Process serial commands (both legacy and JSON)
  agvControl.processSerialCommand();
  
  // Update motor control and sensors
  agvControl.update();
}

// Interrupt handlers for encoders
void rightEncoderISR() {
  rightMotor.encoderCallback();
}

void leftEncoderISR() {
  leftMotor.encoderCallback();
}