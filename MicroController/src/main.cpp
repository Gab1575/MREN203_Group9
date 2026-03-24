#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"
#include "parse.h"

double Lspeed;
double Rspeed; 
InternalIMU::IMUData imuData;

InternalIMU imu;
movement move;
encoders enc;
parse parser;

void sendDataToPi();
unsigned long previousMillis = 0;
const long interval = 20; // 20ms interval = 50 Hz transmission rate
double currentLinVel = 0.0;
double currentAngVel = 0.0;

void setup() {
  Serial.begin(1000000);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
  move.startup();
  enc.startup();
  parser.startup();
}

void loop() {
  unsigned long currentMillis = millis();
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    imuData = imu.read();
    enc.run();
    sendDataToPi(); 
    if (parser.run(currentLinVel, currentAngVel)) {
      move.move(currentAngVel, currentLinVel, 0.1);
    }
  }
} 

void sendDataToPi() {
  // order of data: gyroZ, accelX, accelY, accelZ, omega_L, omega_R
  Serial.print(imuData.gyroZ, 3); Serial.print(","); 
  Serial.print(imuData.accelX, 3); Serial.print(",");
  Serial.print(imuData.accelY, 3); Serial.print(",");
  Serial.print(imuData.accelZ, 3); Serial.print(",");
  Serial.print(enc.omega_L, 3); Serial.print(",");
  Serial.println(enc.omega_R, 3); 
}