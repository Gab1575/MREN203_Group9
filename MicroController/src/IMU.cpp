#include "IMU.h"

void InternalIMU::startup() {
  if (!IMU.begin()) {
    Serial.print("Failed to initialize IMU :(");
    while (1) {
      delay(10);
    }
  }
  Serial.println("On-board IMU initialized");

  // Run the calibration routine immediately after startup
  calibrate();
}

void InternalIMU::calibrate() {
  Serial.println("Calibrating IMU... DO NOT MOVE ROVER!");
  long num_samples = 500;
  float sumX = 0, sumY = 0, sumZ = 0;
  float gx, gy, gz;

  for (int i = 0; i < num_samples; i++) {
    // Wait until new data is physically available on the sensor
    while (!IMU.gyroscopeAvailable()) {
      delay(1); 
    }
    IMU.readGyroscope(gx, gy, gz);
    sumX += gx;
    sumY += gy;
    sumZ += gz;
  }

  // Save the average resting noise for all 3 axes
  gyroX_offset = sumX / num_samples;
  gyroY_offset = sumY / num_samples;
  gyroZ_offset = sumZ / num_samples;
  
  Serial.println("Calibration complete.");
}

InternalIMU::IMUData InternalIMU::read() {
  InternalIMU::IMUData data;
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;

  if (IMU.accelerationAvailable()) IMU.readAcceleration(ax, ay, az);
  if (IMU.gyroscopeAvailable()) IMU.readGyroscope(gx, gy, gz);

  // 1. Apply Gyro Calibration Offsets
  gx -= gyroX_offset;
  gy -= gyroY_offset;
  gz -= gyroZ_offset;

  // 2. Apply Gyro Deadband Filter (Force to 0.0 if movement is tiny)
  if (abs(gx) < DEADBAND_THRESHOLD) gx = 0.0;
  if (abs(gy) < DEADBAND_THRESHOLD) gy = 0.0;
  if (abs(gz) < DEADBAND_THRESHOLD) gz = 0.0;

  // 3. Convert Gyro from deg/s to rad/s for ROS 2
  data.gyroX = gx * DEG_TO_RAD;
  data.gyroY = gy * DEG_TO_RAD;
  data.gyroZ = gz * DEG_TO_RAD;

  // 4. Convert Accel from g-force to m/s^2 for ROS 2
  data.accelX = ax * G_TO_MS2;
  data.accelY = ay * G_TO_MS2;
  data.accelZ = az * G_TO_MS2;

  return data;
}

void InternalIMU::print() {
  // Grab the heavily processed data from our new read() function
  InternalIMU::IMUData data = read();

  Serial.print("Accel X: "); Serial.print(data.accelX, 3);
  Serial.print(" \tY: "); Serial.print(data.accelY, 3);
  Serial.print(" \tZ: "); Serial.print(data.accelZ, 3);
  Serial.println(" m/s^2 ");

  Serial.print("Gyro X: "); Serial.print(data.gyroX, 4);
  Serial.print(" \tY: "); Serial.print(data.gyroY, 4);
  Serial.print(" \tZ: "); Serial.print(data.gyroZ, 4);
  Serial.println(" rad/s ");

  Serial.println("------------------------------------------------");
  // Note: Only use print() for debugging, the 500ms delay here will choke ROS 2 
  // if you call it inside your main loop!
  delay(500); 
}