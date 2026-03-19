//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"

double Lspeed;
double Rspeed; 
InternalIMU::IMUData imuData;

InternalIMU imu;
movement move;
encoders enc;

void sendDataToPi();

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
  move.startup();
  enc.startup();
}
void loop() {
  imuData = imu.read();
  //move.move(0,200, 0.1); // Move forward at 200 mm/s
  enc.run();
  sendDataToPi(); //calls function below, might need delay afterwards
}

//Publish data from the serial monitor for the pi to read
//Data includes IMU gyro and accel, and encoder angular velocity
void sendDataToPi() {
  //order of data: gyroX, gyroY, gyroZ, accelX, accelY, accelZ, omega_L, omega_R
  Serial.print(imuData.gyroZ); Serial.print(","); // Assuming gyroZ is the yaw rate
  Serial.print(imuData.accelX); Serial.print(",");
  Serial.print(imuData.accelY); Serial.print(",");
  Serial.print(imuData.accelZ); Serial.print(",");
  Serial.print(enc.omega_L); Serial.print(",");
  Serial.println(enc.omega_R); //newline char here to signify end of data packet
}