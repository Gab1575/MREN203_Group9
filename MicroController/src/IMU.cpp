#include "IMU.h"

void IMU::startup() {

  Wire.begin(SDA, SCL);

  if (!sox.begin_I2C()) {
    Serial.println("Failed to find LSM6DSOX");
    while (1) { delay(10); }
  }
  Serial.println("LSM6DSOX Found");

  if (!lis3mdl.begin_I2C()) {
    Serial.println("Failed to find LIS3MDL");
    while (1) { delay(10); }
  }
  Serial.println("LIS3MDL Found");

sox.setAccelRange(LSM6DS_ACCEL_RANGE_4_G);
sox.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

sox.setAccelDataRate(LSM6DS_RATE_104_HZ);
sox.setGyroDataRate(LSM6DS_RATE_104_HZ);

lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
}

IMUData IMU::read() {
  sox.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);
    IMUData data;
    data.accelX = accel.acceleration.x;
    data.accelY = accel.acceleration.y;
    data.accelZ = accel.acceleration.z;
    data.gyroX = gyro.gyro.x;
    data.gyroY = gyro.gyro.y;
    data.gyroZ = gyro.gyro.z;
    data.magX = mag.magnetic.x;
    data.magY = mag.magnetic.y;
    data.magZ = mag.magnetic.z;
    return data;
}

void IMU::print() {
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x);
  Serial.print(" \tY: "); Serial.print(accel.acceleration.y);
  Serial.print(" \tZ: "); Serial.print(accel.acceleration.z);
  Serial.println(" m/s^2 ");

  Serial.print("Gyro X: "); Serial.print(gyro.gyro.x);
  Serial.print(" \tY: "); Serial.print(gyro.gyro.y);
  Serial.print(" \tZ: "); Serial.print(gyro.gyro.z);
  Serial.println(" radians/s ");

  Serial.print("Mag X: "); Serial.print(mag.magnetic.x);
  Serial.print(" \tY: "); Serial.print(mag.magnetic.y);
  Serial.print(" \tZ: "); Serial.print(mag.magnetic.z);
  Serial.println(" uTesla ");

  Serial.println("------------------------------------------------");
  delay(500);
}