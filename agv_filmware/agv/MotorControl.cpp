#include "MotorControl.h"
MotorControl *MotorControl::instance = nullptr;
MotorControl::MotorControl(double rightRadiantTarget, double leftRadiantTarget, char rightDir, char leftDir)
{
  instance = this;
  right_encoder_counter = 0;
  left_encoder_counter = 0;
  right_wheel_meas_vel = 0.0;
  left_wheel_meas_vel = 0.0;

  // PID Variables
  right_setpoint = rightRadiantTarget; // rad.s target
  left_setpoint = leftRadiantTarget;   // rad.s target
  right_pwm_output = 0;
  left_pwm_output = 0;

  // PID gains tunning
  Kp = 50;
  Ki = 10;
  Kd = 1;

  right_integral = 0;
  left_integral = 0;
  right_last_error = 0;
  left_last_error = 0;

  // Motor Direction
  right_motor_direction = rightDir;
  left_motor_direction = leftDir;
}
void MotorControl::setup()
{
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  pinMode(right_encoder_phaseB, INPUT);
  pinMode(left_encoder_phaseB, INPUT);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderCallbackStatic, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderCallbackStatic, RISING);
}
void MotorControl::run(double rightRadiantTarget, double leftRadiantTarget, char rightDir, char leftDir)
{
  right_setpoint = rightRadiantTarget;
  left_setpoint = leftRadiantTarget;
  right_motor_direction = rightDir;
  left_motor_direction = leftDir;
  // PID loop setiap 100ms
  double dt = 0.1;

  noInterrupts();
  double right_count = right_encoder_counter;
  double left_count = left_encoder_counter;
  right_encoder_counter = 0;
  left_encoder_counter = 0;
  interrupts();

  right_wheel_meas_vel = (10 * right_count * (60.0 / 385.0)) * 0.10472;
  left_wheel_meas_vel = (10 * left_count * (60.0 / 385.0)) * 0.10472;

  // === PID Motor Kanan ===
  double right_error = right_setpoint - right_wheel_meas_vel;
  right_integral += right_error * dt;
  double right_derivative = (right_error - right_last_error) / dt;
  right_pwm_output = (Kp * right_error) + (Ki * right_integral) + (Kd * right_derivative);
  right_last_error = right_error;
  right_pwm_output = constrain(right_pwm_output, 0, 255);

  // === PID Motor Kiri ===
  double left_error = left_setpoint - left_wheel_meas_vel;
  left_integral += left_error * dt;
  double left_derivative = (left_error - left_last_error) / dt;
  left_pwm_output = (Kp * left_error) + (Ki * left_integral) + (Kd * left_derivative);
  left_last_error = left_error;
  left_pwm_output = constrain(left_pwm_output, 0, 255);

  if (right_motor_direction == 'p')
  {
    digitalWrite(L298N_in1, LOW);
    digitalWrite(L298N_in2, HIGH);
  }
  else
  {
    digitalWrite(L298N_in1, HIGH);
    digitalWrite(L298N_in2, LOW);
  }
  analogWrite(L298N_enA, right_pwm_output);
  if (left_motor_direction == 'p')
  {
    digitalWrite(L298N_in3, LOW);
    digitalWrite(L298N_in4, HIGH);
  }
  else
  {
    digitalWrite(L298N_in3, HIGH);
    digitalWrite(L298N_in4, LOW);
  }
  analogWrite(L298N_enB, left_pwm_output);
  delay(100);
}
void MotorControl::rightEncoderCallbackStatic()
{
  if (instance)
  {
    instance->rightEncoderCallback();
  }
}
void MotorControl::leftEncoderCallbackStatic()
{
  if (instance)
  {
    instance->leftEncoderCallback();
  }
}
void MotorControl::rightEncoderCallback()
{
  right_encoder_counter++;
}
void MotorControl::leftEncoderCallback()
{
  left_encoder_counter++;
}
double MotorControl::getRightVelocity()
{
  return right_wheel_meas_vel;
}
double MotorControl::getLeftVelocity()
{
  return left_wheel_meas_vel;
}
char MotorControl::getRightDir()
{
  return right_motor_direction;
}
char MotorControl::getLeftDir()
{
  return left_motor_direction;
}