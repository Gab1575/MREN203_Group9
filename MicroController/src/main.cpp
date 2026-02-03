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
  move.move(0,200, 0.1); // Move forward at 200 mm/s
  enc.run();
}