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
unsigned long previousMillis = 0;
const long interval = 20; // 20ms interval = 50 Hz transmission rate

void setup() {
  Serial.begin(1000000);
  if(!Serial) {
    while (1); // Wait for Serial to be ready
  }
  imu.startup();
  move.startup();
  enc.startup();
}

void loop() {
  unsigned long currentMillis = millis(); // Added: Grab the current time
  
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    
    imuData = imu.read();
    //move.move(0,200, 0.1); // Move forward at 200 mm/s
    enc.run();
    
    sendDataToPi(); 
  }
} // Added: Missing closing brace for the loop() function

//Publish data from the serial monitor for the pi to read
//Data includes IMU gyro and accel, and encoder angular velocity
void sendDataToPi() {
  // Corrected comment to match the actual data being sent
  // order of data: gyroZ, accelX, accelY, accelZ, omega_L, omega_R
  Serial.print(imuData.gyroZ, 3); Serial.print(","); 
  Serial.print(imuData.accelX, 3); Serial.print(",");
  Serial.print(imuData.accelY, 3); Serial.print(",");
  Serial.print(imuData.accelZ, 3); Serial.print(",");
  Serial.print(enc.omega_L, 3); Serial.print(",");
  Serial.println(enc.omega_R, 3); //newline char here to signify end of data packet
}