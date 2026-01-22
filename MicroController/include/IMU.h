//IMU.h

//PINS SDA
//PINS SCL

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LIS3MDL.h>

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
    void IMU::print();
private:
    static const int IMU_ADDRESS = 0x68;
    int16_t accelX, accelY, accelZ;

    Adafruit_LSM6DSOX sox;
    Adafruit_LIS3MDL lis3mdl;

    sensors_event_t accel, gyro, temp;
    sensors_event_t mag;
};

#endif // IMU_H
