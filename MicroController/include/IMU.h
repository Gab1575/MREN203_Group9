// IMU.h

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

class InternalIMU {
public:
    struct IMUData {
        float accelX;
        float accelY;
        float accelZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    void startup();
    IMUData read();
    void print();
};

#endif // IMU_H

