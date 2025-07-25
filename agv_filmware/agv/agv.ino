#include "MotorControl.h"
#include "imu.h"
#include "barrometer.h"
#include <ArduinoJson.h>


MotorControl motor(10.0, 10.0, 'p', 'p');
Imu imu;
Barrometer barrometer;
JsonDocument docRead;
JsonDocument docWrite;
double right_vel = 0;
double left_vel = 0;
String right_dir = "p";
String left_dir = "p";
void setup() {
  motor.setup();
  imu.init();
  Serial.begin(115200);
  barrometer.setup();

}
void loop() {
  if(Serial.available()>0) {
    String input = Serial.readString();
    deserializeJson(docRead, input);
    right_vel = docRead["right_velocity"];
    left_vel = docRead["left_velocity"];
    String right_dir = docRead["right_direction"];
    String left_dir = docRead["left_direction"];
  }
  

  motor.run(right_vel, left_vel, right_dir[0], left_dir[0]);
  // Serial.print(", Aglr Velocity Z: ");
  // Serial.print(imu.angularVelocityZaxis());
  // Serial.print(", Euler Z: ");
  // Serial.println(imu.eulerAngleZ());
  transfer();
  //delay(100);
}
void transfer() {
  // Mengisi data ke dalam JSON
  docWrite["imu"] = imu.angularVelocityZaxis();
  docWrite["right_velocity"] = motor.getRightVelocity();
  docWrite["left_velocity"] = motor.getLeftVelocity();
  
  // Mengubah char menjadi String
  docWrite["right_direction"] = String(motor.getRightDir());
  docWrite["left_direction"] = String(motor.getLeftDir());
  // Serialisasi JSON ke dalam char array
  char output[256];
  serializeJson(docWrite, output);
  Serial.println(output);
  //delay(100);
  // Serial.print(imu.angularVelocityZaxis());
  // Serial.print(", ");
  // Serial.print(motor.getRightVelocity());
  // Serial.print(", ");
  // Serial.print(motor.getLeftVelocity());
  // Serial.print(", ");
  // Serial.print(motor.getRightDir());
  // Serial.print(", ");
  // Serial.print(motor.getLeftDir());
  // Serial.println();
}
