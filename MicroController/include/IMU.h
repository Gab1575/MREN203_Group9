//IMU.h

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

#define SDA 8 
#define SCL 9

struct IMUData {
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    float magX;
    float magY;
    float magZ;
};

class IMU {
public:
    void startup();
    IMUData read();
    void print();
private:
    static const int IMU_ADDRESS = 0x68;
    int16_t accelX, accelY, accelZ;

    Adafruit_LSM6DSOX sox;
    Adafruit_LIS3MDL lis3mdl;

    sensors_event_t accel, gyro, temp;
    sensors_event_t mag;
};

#endif // IMU_H
