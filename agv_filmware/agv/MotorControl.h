#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "Arduino.h"
#define L298N_enA 4  
#define L298N_enB 6
#define L298N_in1 23
#define L298N_in2 25
#define L298N_in3 27
#define L298N_in4 29 

#define right_encoder_phaseA 3  
#define right_encoder_phaseB 5
#define left_encoder_phaseA 19
#define left_encoder_phaseB 9  

class MotorControl{
  private: 
    volatile unsigned int right_encoder_counter;
    volatile unsigned int left_encoder_counter;
    static void rightEncoderCallbackStatic();
    static void leftEncoderCallbackStatic();
    void rightEncoderCallback();
    void leftEncoderCallback();

    double right_wheel_meas_vel;  
    double left_wheel_meas_vel;
    double right_setpoint;
    double left_setpoint;
    double right_pwm_output;
    double left_pwm_output;
    double Kp;
    double Ki;
    double Kd;
    double right_integral;
    double left_integral;
    double right_last_error;
    double left_last_error;
    char right_motor_direction;
    char left_motor_direction;
  public:
    static MotorControl* instance;
    MotorControl(double rightRadiantTarget, double leftRadiantTarget, char rightDir, char leftDir);
    void setup();
    void run(double rightRadiantTarget, double leftRadiantTarget, char rightDir, char leftDir);
    double getRightVelocity();
    double getLeftVelocity();
    char getRightDir();
    char getLeftDir();
};
#endif