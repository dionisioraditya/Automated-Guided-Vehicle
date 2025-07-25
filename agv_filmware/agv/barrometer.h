#ifndef BARROMETER_H
#define BARROMETER_H

#include <DFRobot_BMP280.h>
#include <Wire.h>
#include <Arduino.h>

#define SEA_LEVEL_PRESSURE 1015.0f

typedef DFRobot_BMP280_IIC BMP;

class Barrometer {
private:
    BMP bmp;
    
public:
    // Constructor
    Barrometer() : bmp(&Wire, BMP::eSdoLow) {}
    
    // Initialize the barometer
    void printLastOperateStatus(BMP::eStatus_t eStatus)
        {
        switch(eStatus) {
        case BMP::eStatusOK:    Serial.println("everything ok"); break;
        case BMP::eStatusErr:   Serial.println("unknow error"); break;
        case BMP::eStatusErrDeviceNotDetected:    Serial.println("device not detected"); break;
        case BMP::eStatusErrParameter:    Serial.println("parameter error"); break;
        default: Serial.println("unknow status"); break;
        }
    }
    void setup() {
        Serial.begin(115200);
        bmp.reset();
        Serial.println("bmp read data test");
        while(bmp.begin() != BMP::eStatusOK) {
            Serial.println("bmp begin faild");
            printLastOperateStatus(bmp.lastOperateStatus);
            delay(2000);
        }
        Serial.println("bmp begin success");
        delay(100);
    }
    
    // Get temperature in Celsius
    float getTemperature() {
        return bmp.getTemperature();
    }
    
    // Get pressure in Pa
    uint32_t getPressure() {
        return bmp.getPressure();
    }
    
    // Calculate altitude based on pressure
    float getAltitude() {
        uint32_t pressure = getPressure();
        return bmp.calAltitude(SEA_LEVEL_PRESSURE, pressure);
    }
    
    // Get the last operation status
    BMP::eStatus_t getLastStatus() {
        return bmp.lastOperateStatus;
    }
    
    // Print the status message
    void printStatus() {
        BMP::eStatus_t status = getLastStatus();
        switch(status) {
            case BMP::eStatusOK:
                Serial.println("Everything OK");
                break;
            case BMP::eStatusErr:
                Serial.println("Unknown error");
                break;
            case BMP::eStatusErrDeviceNotDetected:
                Serial.println("Device not detected");
                break;
            case BMP::eStatusErrParameter:
                Serial.println("Parameter error");
                break;
            default:
                Serial.println("Unknown status");
                break;
        }
    }
};

#endif