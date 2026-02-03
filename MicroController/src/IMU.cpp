#include "IMU.h"

void InternalIMU::startup() {
  if (!IMU.begin()) {
    Serial.print("Failed to initialize IMU :(");
    while (1) {
      delay(10);
    }
  }
  Serial.println("On-board IMU initialized");
}

InternalIMU::IMUData InternalIMU::read() {
  InternalIMU::IMUData data;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx, gy, gz);

  data.accelX = ax;
  data.accelY = ay;
  data.accelZ = az;
  data.gyroX = gx;
  data.gyroY = gy;
  data.gyroZ = gz;
  return data;
}

void InternalIMU::print() {
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx, gy, gz);

  Serial.print("Accel X: "); Serial.print(ax);
  Serial.print(" \tY: "); Serial.print(ay);
  Serial.print(" \tZ: "); Serial.print(az);
  Serial.println(" m/s^2 ");

  Serial.print("Gyro X: "); Serial.print(gx);
  Serial.print(" \tY: "); Serial.print(gy);
  Serial.print(" \tZ: "); Serial.print(gz);
  Serial.println(" rad/s ");

  Serial.println("------------------------------------------------");
  delay(500);
}