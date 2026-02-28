//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"
#include "wifiCom.h"

//ENCODERS
// SIGNAL_A 2
// SIGNAL_B 3
// SIGNAL_C 4
// SIGNAL_D 5

//MOTOR PINS
// EA 6  
// EB 11 

// I1 8 
// I2 7 
// I4 9 
// I3 12 



double Lspeed;
double Rspeed; 
InternalIMU::IMUData imuData;

InternalIMU imu;
movement move;
encoders enc;
wifiCom wifi;

double prevTime = 0;

void setup() {
  Serial.begin(115200);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
  //wifi.startup();
  move.startup();
  enc.startup();
  move.calibrateFeedforward();
}
void loop() {
  enc.run(); 
  
  double currentTime = millis();
  double dt = (currentTime - prevTime) / 1000.0; 
  
  if (dt >= 0.05) { 
    prevTime = currentTime;
    
    imuData = imu.read();
    
    move.move(0, 9.0, dt, enc.omega_L, enc.omega_R, imuData.gyroZ);
    
    enc.print(); // Optional: Keep this inside the timed loop so it doesn't flood the Serial monitor

  }
}