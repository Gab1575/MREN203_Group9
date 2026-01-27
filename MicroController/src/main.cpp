//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"

double Lspeed;
double Rspeed; 
IMU::IMUData imuData;

IMU imu;
movement move;
encoders enc;

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  //imu.startup();
  //move.startup();
  enc.startup();
}
void loop() {
  //imuData = imu.read();
  //imu.print();
  //delay(100);
  //move.forward(200); // Move forward at 0.2 m/s
  enc.run();
  enc.print();
  delay(100);
}