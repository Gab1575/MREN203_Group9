//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"

double Lspeed;
double Rspeed; 
IMUData imuData;

IMU imu;

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
}
void loop() {
  imuData = imu.read();
  imu.print();
  delay(100);
}