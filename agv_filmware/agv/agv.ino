#include "MotorControl.h"
#include "imu.h"
#include "barrometer.h"
#include <ArduinoJson.h>


MotorControl motor(10.0, 10.0, 'p', 'p');
Imu imu;
Barrometer barrometer;
JsonDocument doc;

void setup() {
  motor.setup();
  imu.init();
  Serial.begin(115200);
  barrometer.setup();

}
void loop() {
  JsonDocument doc;
  String input = Serial.readString();
  deserializeJson(doc, input);
  double right_vel = doc["right_velocity"];
  double left_vel = doc["left_velocity"];
  String right_dir = doc["right_direction"];
  String left_dir = doc["left_direction"];

  motor.run(right_vel, left_vel, right_dir[0], left_dir[0]);
  // Serial.print(", Aglr Velocity Z: ");
  // Serial.print(imu.angularVelocityZaxis());
  // Serial.print(", Euler Z: ");
  // Serial.println(imu.eulerAngleZ());
  transfer();

}
void transfer() {
  JsonDocument doc;
  // Mengisi data ke dalam JSON
  doc["imu"] = imu.angularVelocityZaxis();
  doc["right_velocity"] = motor.getRightVelocity();
  doc["left_velocity"] = motor.getLeftVelocity();
  
  // Mengubah char menjadi String
  doc["right_direction"] = String(motor.getRightDir());
  doc["left_direction"] = String(motor.getLeftDir());
  // Serialisasi JSON ke dalam char array
  char output[256];
  serializeJson(doc, output);
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
