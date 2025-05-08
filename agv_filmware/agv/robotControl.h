#ifndef ROBOT_CONTROL_H
#define ROBOT_CONTROL_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "motor.h"
#include "imu.h"
#include "barrometer.h"

class RobotControl {
private:
    Motor* rightMotor;
    Motor* leftMotor;
    Imu* imu;  // Tambahkan pointer ke objek IMU
    Barrometer* bmp;  // Tambahkan pointer ke objek Barometer
    unsigned long lastMillis;
    unsigned long interval;
    unsigned long lastSensorSendMillis;  // Interval untuk kirim data sensor
    unsigned long sensorSendInterval;    // Interval kirim data sensor
    
    // Serial command parsing
    bool isRightWheelCmd;
    bool isLeftWheelCmd;
    char value[6];
    uint8_t valueIdx;
    bool isCmdComplete;
    
public:
    // Default constructor
    RobotControl() {
        rightMotor = nullptr;
        leftMotor = nullptr;
        imu = nullptr;
        bmp = nullptr;
        lastMillis = 0;
        interval = 100;
        lastSensorSendMillis = 0;
        sensorSendInterval = 50; // 50ms = 20Hz untuk data sensor
        isRightWheelCmd = false;
        isLeftWheelCmd = false;
        valueIdx = 0;
        isCmdComplete = false;
        
        // Initialize value array
        value[0] = '0';
        value[1] = '0';
        value[2] = '.';
        value[3] = '0';
        value[4] = '0';
        value[5] = '\0';
    }
    
    // Constructor with parameters
    RobotControl(Motor* rightMotor, Motor* leftMotor, Imu* imu = nullptr, Barrometer* bmp = nullptr, unsigned long interval = 100) {
        this->rightMotor = rightMotor;
        this->leftMotor = leftMotor;
        this->imu = imu;
        this->bmp = bmp;
        this->interval = interval;
        
        lastMillis = 0;
        lastSensorSendMillis = 0;
        sensorSendInterval = 50; // 50ms = 20Hz
        isRightWheelCmd = false;
        isLeftWheelCmd = false;
        valueIdx = 0;
        isCmdComplete = false;
        
        // Initialize value array
        value[0] = '0';
        value[1] = '0';
        value[2] = '.';
        value[3] = '0';
        value[4] = '0';
        value[5] = '\0';
    }
    
    // Set motors
    void setMotors(Motor* rightMotor, Motor* leftMotor) {
        this->rightMotor = rightMotor;
        this->leftMotor = leftMotor;
    }
    
    // Set sensors
    void setSensors(Imu* imu, Barrometer* bmp) {
        this->imu = imu;
        this->bmp = bmp;
    }
    
    // Initialize robot
    void init() {
        // Set up serial for communication
        Serial.begin(115200);
    }
    
    // Process serial commands (Legacy format)
    void processSerialCommand() {
        if (Serial.available()) {
            char chr = Serial.read();
            
            // Check if this is a JSON command starting with 'J:'
            if (chr == 'J') {
                if (Serial.read() == ':') {
                    // Process as JSON
                    processJsonCommand();
                    return;
                }
            }
            
            // Legacy command parsing
            if (chr == 'r') {
                isRightWheelCmd = true;
                isLeftWheelCmd = false;
                valueIdx = 0;
                isCmdComplete = false;
            }
            else if (chr == 'l') {
                isRightWheelCmd = false;
                isLeftWheelCmd = true;
                valueIdx = 0;
            }
            else if (chr == 'p') {
                if (isRightWheelCmd && rightMotor) {
                    rightMotor->setDirection(true);
                } else if (isLeftWheelCmd && leftMotor) {
                    leftMotor->setDirection(true);
                }
            }
            else if (chr == 'n') {
                if (isRightWheelCmd && rightMotor) {
                    rightMotor->setDirection(false);
                } else if (isLeftWheelCmd && leftMotor) {
                    leftMotor->setDirection(false);
                }
            }
            else if (chr == ',') {
                if (isRightWheelCmd && rightMotor) {
                    rightMotor->setVelocity(atof(value));
                } else if (isLeftWheelCmd && leftMotor) {
                    leftMotor->setVelocity(atof(value));
                    isCmdComplete = true;
                }
                // Reset for next command
                valueIdx = 0;
                value[0] = '0';
                value[1] = '0';
                value[2] = '.';
                value[3] = '0';
                value[4] = '0';
                value[5] = '\0';
            }
            else {
                if (valueIdx < 5) {
                    value[valueIdx] = chr;
                    valueIdx++;
                }
            }
        }
    }
    
    // Process JSON commands from serial
    void processJsonCommand() {
        String jsonStr = Serial.readStringUntil('\n');
        
        // Parse JSON
        StaticJsonDocument<256> jsonDoc;
        DeserializationError error = deserializeJson(jsonDoc, jsonStr);
        
        if (error) {
            // JSON parsing error
            sendJsonError("Parse error");
            return;
        }
        
        // Check if this is a motor command
        if (jsonDoc.containsKey("motor")) {
            JsonObject motor = jsonDoc["motor"];
            
            if (motor.containsKey("right") && rightMotor) {
                JsonObject right = motor["right"];
                
                if (right.containsKey("velocity")) {
                    double velocity = right["velocity"];
                    rightMotor->setVelocity(velocity);
                }
                
                if (right.containsKey("direction")) {
                    bool direction = right["direction"].as<String>() == "p";
                    rightMotor->setDirection(direction);
                }
            }
            
            if (motor.containsKey("left") && leftMotor) {
                JsonObject left = motor["left"];
                
                if (left.containsKey("velocity")) {
                    double velocity = left["velocity"];
                    leftMotor->setVelocity(velocity);
                }
                
                if (left.containsKey("direction")) {
                    bool direction = left["direction"].as<String>() == "p";
                    leftMotor->setDirection(direction);
                }
            }
            
            // Send acknowledgment
            sendJsonAck("Motor command received");
        }
        
        // Check if this is a sensor request
        if (jsonDoc.containsKey("request")) {
            String request = jsonDoc["request"].as<String>();
            
            if (request == "sensors") {
                // Send immediate sensor data
                sendSensorData();
            }
            
            if (request == "set_rate" && jsonDoc.containsKey("rate")) {
                int rate = jsonDoc["rate"]; // Hz
                if (rate > 0) {
                    sensorSendInterval = 1000 / rate; // Convert Hz to ms
                } else {
                    sensorSendInterval = 0; // Disable automatic sending
                }
                
                String message = "Sensor rate set to " + String(rate) + " Hz";
                sendJsonAck(message.c_str());
            }
        }
    }
    
    // Update motor measurements and commands
    void update() {
        unsigned long currentMillis = millis();
        
        // Update motors
        if (rightMotor && leftMotor && currentMillis - lastMillis >= interval) {
            // Update measurements
            rightMotor->updateMeasurement(interval);
            leftMotor->updateMeasurement(interval);
            
            // Compute PID
            rightMotor->computePID();
            leftMotor->computePID();
            
            // Send feedback
            sendFeedbackJson();
            
            // Apply commands
            rightMotor->applyCommand();
            leftMotor->applyCommand();
            
            // Reset encoders
            rightMotor->resetEncoder();
            leftMotor->resetEncoder();
            
            lastMillis = currentMillis;
        }
        
        // Send sensor data periodically if enabled
        if (sensorSendInterval > 0 && currentMillis - lastSensorSendMillis >= sensorSendInterval) {
            sendSensorData();
            lastSensorSendMillis = currentMillis;
        }
    }
    
    // Send sensor data (IMU + Barometer)
    void sendSensorData() {
        StaticJsonDocument<512> jsonDoc;
        
        jsonDoc["ts"] = millis();
        
        // IMU data if available
        if (imu) {
            JsonObject imuObj = jsonDoc.createNestedObject("imu");
            
            // Angular velocity (gyro)
            JsonObject gyro = imuObj.createNestedObject("gyro");
            gyro["x"] = imu->angularVelocityXaxis();
            gyro["y"] = imu->angularVelocityYaxis();
            gyro["z"] = imu->angularVelocityZaxis();
            
            // Euler angles (orientation)
            JsonObject euler = imuObj.createNestedObject("euler");
            euler["x"] = imu->eulerAngleX();
            euler["y"] = imu->eulerAngleY();
            euler["z"] = imu->eulerAngleZ();
        }
        
        // Barometer data if available
        if (bmp) {
            JsonObject bmpObj = jsonDoc.createNestedObject("baro");
            bmpObj["temp"] = bmp->getTemperature();
            bmpObj["press"] = bmp->getPressure();
            bmpObj["alt"] = bmp->getAltitude();
        }
        
        // Send with sensor identifier
        Serial.print("S:");  // Sensor data identifier
        serializeJson(jsonDoc, Serial);
        Serial.println();
    }
    
    // Send JSON feedback (motor data)
    void sendFeedbackJson() {
        if (!rightMotor || !leftMotor) return;
        
        StaticJsonDocument<256> jsonDoc;
        
        jsonDoc["ts"] = millis();
        
        JsonObject right = jsonDoc.createNestedObject("r");
        right["v"] = rightMotor->getMeasuredVelocity();
        right["d"] = rightMotor->getWheelSign();
        right["c"] = rightMotor->getCommand();
        
        JsonObject left = jsonDoc.createNestedObject("l");
        left["v"] = leftMotor->getMeasuredVelocity();
        left["d"] = leftMotor->getWheelSign();
        left["c"] = leftMotor->getCommand();
        
        Serial.print("D:");  // Data identifier
        serializeJson(jsonDoc, Serial);
        Serial.println();
    }
    
    // Legacy feedback method
    void sendFeedback() {
        if (!rightMotor || !leftMotor) return;
        
        String encoderRead = "r" + rightMotor->getWheelSign() + 
                             String(rightMotor->getMeasuredVelocity()) + 
                             ",l" + leftMotor->getWheelSign() + 
                             String(leftMotor->getMeasuredVelocity()) + ",";
        Serial.println(encoderRead);
    }
    
    // Send JSON error message
    void sendJsonError(const char* message) {
        StaticJsonDocument<128> jsonDoc;
        jsonDoc["error"] = message;
        
        Serial.print("E:");  // Error identifier
        serializeJson(jsonDoc, Serial);
        Serial.println();
    }
    
    // Send JSON acknowledgment
    void sendJsonAck(const char* message) {
        StaticJsonDocument<128> jsonDoc;
        jsonDoc["ack"] = message;
        
        Serial.print("A:");  // Acknowledgment identifier
        serializeJson(jsonDoc, Serial);
        Serial.println();
    }
};

#endif