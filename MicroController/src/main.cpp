#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"
#include "wifiCom.h"

double Lspeed;
double Rspeed; 
InternalIMU::IMUData imuData;

InternalIMU imu;
movement move;
encoders enc;
wifiCom wifi;

// TWO separate timers
unsigned long prevFastTime = 0;
unsigned long prevSensorTime = 0;

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
  move.startup();
  enc.startup();
  //move.calibrateFeedforward();
}

void loop() {  
  unsigned long currentMillis = millis();
  
  double dt = (currentMillis - prevFastTime) / 1000.0; 
  
  if (dt > 0) {
    move.move(0, 10, dt, enc.omega_L, enc.omega_R, imuData.gyroZ);
    prevFastTime = currentMillis; 
  }

  if (currentMillis - prevSensorTime >= 50) { 
    prevSensorTime = currentMillis;
    imuData = imu.read();
    enc.run(); 
    enc.print();
  }
}