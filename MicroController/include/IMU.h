// IMU.h

#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Arduino_LSM6DS3.h>

class InternalIMU {
private:
    // Calibration offsets
    float gyroX_offset = 0.0;
    float gyroY_offset = 0.0;
    float gyroZ_offset = 0.0;

    // Math Constants
    const float G_TO_MS2 = 9.80665;
    const float DEADBAND_THRESHOLD = 1.5; // deg/s

    // Internal helper function
    void calibrate();

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