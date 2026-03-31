#include <Arduino.h>
#include "encoders.h"
#include "movement.h"
#include "IMU.h"
#include "parse.h"

double GYRO_MULTIPLIER = 2.3; // (Keep whatever number you tuned this to)
double gyroZ_bias = 0.0;      // NEW: Stores the calibration offset

double Lspeed;
double Rspeed; 
InternalIMU::IMUData imuData;

InternalIMU imu;
movement move;
encoders enc;
parse parser;

void sendDataToPi();
unsigned long previousMillis = 0;
const long interval = 5; // CHANGED: 5ms interval = 200 Hz
double currentLinVel = 0.0;
double currentAngVel = 0.0;

void setup() {
  Serial.begin(1000000);
  imu.startup();
  move.startup();
  enc.startup();
  parser.startup();

  // ==========================================
  // GYRO CALIBRATION ROUTINE
  // DO NOT touch or move the robot while it boots!
  // ==========================================
  double bias_sum = 0.0;
  int num_samples = 500;
  
  for (int i = 0; i < num_samples; i++) {
    imuData = imu.read();
    bias_sum += imuData.gyroZ;
    delay(2); // Wait 2ms between samples to get fresh data
  }
  
  gyroZ_bias = bias_sum / (double)num_samples;
}
unsigned long previousControlMillis = 0;
unsigned long previousTelemetryMillis = 0;
unsigned long lastCommandTime = 0; // Watchdog timer

const long controlInterval = 5;    // 200 Hz for precise motor control
const long telemetryInterval = 20; // 50 Hz for sending data to the Pi

void loop() {
  unsigned long currentMillis = millis();

  // 1. Read serial continuously. 
  // If parser.run() returns true, a full, new command arrived.
if (parser.run(currentLinVel, currentAngVel)) {
    lastCommandTime = currentMillis; // Reset the watchdog timer
  }

  // ==========================================
  // SAFETY WATCHDOG
  // If we haven't received a valid command in 500ms, STOP!
  // ==========================================
  if (currentMillis - lastCommandTime > 500) {
    currentLinVel = 0.0;
    currentAngVel = 0.0;
  }

  // ==========================================
  // FAST LOOP: 200Hz Motor Control
  // ==========================================
  if (currentMillis - previousControlMillis >= controlInterval) {
    previousControlMillis = currentMillis;
    
    imuData = imu.read();
    
    // --- APPLY THE CALIBRATION AND MULTIPLIER ---
    // 1. Subtract the bias so stationary = 0.000
    // 2. Multiply by your tuned scale factor
    imuData.gyroZ = (imuData.gyroZ - gyroZ_bias) * GYRO_MULTIPLIER;    
    
    enc.run();

    move.move(currentAngVel, currentLinVel, 0.01, enc.omega_L, enc.omega_R, imuData.gyroZ);
  }

  // ==========================================
  // SLOW LOOP: 50Hz Serial Telemetry
  // ==========================================
  if (currentMillis - previousTelemetryMillis >= telemetryInterval) {
    previousTelemetryMillis = currentMillis;
    
    sendDataToPi(); // Send data at a reasonable rate so the Pi doesn't choke
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