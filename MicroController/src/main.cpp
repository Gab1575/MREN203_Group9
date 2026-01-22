//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"

double Lspeed;
double Rspeed; 
IMUData imuData;

encoders Encoders;
movement Movement;
IMU imu;

void setup() {
  Serial.begin(9600);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }

  Encoders.startup();
  Movement.startup();
  imu.startup();
}
void loop() {
  Movement.forward(200);
  Encoders.run();
  imuData = imu.read();
  imu.print();
  Encoders.print();
  delay(100);
}