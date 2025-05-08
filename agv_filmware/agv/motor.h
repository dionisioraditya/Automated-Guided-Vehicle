#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <PID_v1.h>

class Motor {
private:
    // Pin configurations
    uint8_t enablePin;
    uint8_t in1Pin;
    uint8_t in2Pin;
    uint8_t encoderPhaseA;
    uint8_t encoderPhaseB;
    
    // Encoder variables
    volatile unsigned int encoderCounter;
    String wheelSign;
    bool isForward;
    
    // PID variables
    double commandVelocity;  // Desired velocity (rad/s)
    double measuredVelocity; // Measured velocity (rad/s)
    double motorCommand;     // Output command (0-255)
    double kp, ki, kd;       // PID tuning parameters
    PID* pidController;      // PID controller
    
public:
    // Constructor
    Motor(uint8_t enablePin, uint8_t in1Pin, uint8_t in2Pin, 
          uint8_t encoderPhaseA, uint8_t encoderPhaseB,
          double kp, double ki, double kd) {
        this->enablePin = enablePin;
        this->in1Pin = in1Pin;
        this->in2Pin = in2Pin;
        this->encoderPhaseA = encoderPhaseA;
        this->encoderPhaseB = encoderPhaseB;
        
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
        
        encoderCounter = 0;
        wheelSign = "p";  // Default positive direction
        isForward = true;
        
        commandVelocity = 0.0;
        measuredVelocity = 0.0;
        motorCommand = 0.0;
        
        pidController = new PID(&measuredVelocity, &motorCommand, &commandVelocity, kp, ki, kd, DIRECT);
    }
    
    // Destructor
    ~Motor() {
        delete pidController;
    }
    
    // Initialize motor pins
    void init() {
        // Set pin modes
        pinMode(enablePin, OUTPUT);
        pinMode(in1Pin, OUTPUT);
        pinMode(in2Pin, OUTPUT);
        pinMode(encoderPhaseB, INPUT);
        
        // Set initial direction
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        
        // Enable PID
        pidController->SetMode(AUTOMATIC);
    }
    
    // Set motor direction
    void setDirection(bool forward) {
        if (forward != isForward) {
            // Change direction of rotation
            digitalWrite(in1Pin, HIGH - digitalRead(in1Pin));
            digitalWrite(in2Pin, HIGH - digitalRead(in2Pin));
            isForward = forward;
        }
    }
    
    // Set velocity command
    void setVelocity(double velocity) {
        commandVelocity = velocity;
    }
    
    // Update measured velocity
    void updateMeasurement(unsigned long intervalMs) {
        // Convert pulses to radians/second
        // 385 pulses per revolution, convert to rad/s
        // 60 sec/min * 10 pulses / interval (100ms) = 6000 pulses/min
        measuredVelocity = (10 * encoderCounter * (60.0/385.0)) * 0.10472; // 0.10472 rad/s per rpm
    }
    
    // Compute PID and update motor command
    void computePID() {
        // Compute PID if velocity command is not zero
        if (commandVelocity == 0.0) {
            motorCommand = 0.0;
        } else {
            pidController->Compute();
        }
    }
    
    // Apply motor command
    void applyCommand() {
        analogWrite(enablePin, motorCommand);
    }
    
    // Encoder callback handler
    void encoderCallback() {
        // Update wheel sign based on encoder phase B
        if (digitalRead(encoderPhaseB) == HIGH) {
            wheelSign = (encoderPhaseA == 3) ? "p" : "n"; // Different for left and right motors
        } else {
            wheelSign = (encoderPhaseA == 3) ? "n" : "p"; // Different for left and right motors
        }
        encoderCounter++;
    }
    
    // Get measured velocity
    double getMeasuredVelocity() {
        return measuredVelocity;
    }
    
    // Get wheel sign
    String getWheelSign() {
        return wheelSign;
    }
    
    // Get motor command
    double getCommand() {
        return motorCommand;
    }
    
    // Reset encoder counter
    void resetEncoder() {
        encoderCounter = 0;
    }
    
    // Get encoder counter
    unsigned int getEncoderCounter() {
        return encoderCounter;
    }
};

#endif