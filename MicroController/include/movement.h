// movement.h

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include "IMU.h"
#include "encoders.h"
#include <Arduino.h>
#include <map>
#include <EEPROM.h>

// MACRO-PULSING CONTROLS
#define MIN_PWM 130
#define MACRO_PWM 200
#define MACRO_WINDOW_MS 150  //should be half the encoder sampling rate

// PID CONTROLS:
#define LEFT_KP 2.5
#define LEFT_KD 0.2
#define LEFT_KI 3
#define LEFT_MAX_INTEGRAL 20

#define RIGHT_KP 2.5
#define RIGHT_KD 0.2
#define RIGHT_KI 4
#define RIGHT_MAX_INTEGRAL 20

#define ANGULAR_KP 0
#define ANGULAR_KD 0
#define ANGULAR_KI 0
#define ANGULAR_MAX_INTEGRAL 100.0

#define MAX_ACCELERATION 10.0 // Max change in velocity per second

#define TRACK_WIDTH 0.28

//MOTOR PINS
#define EA 6  // PWM pin for left wheel
#define EB 11 // PWM pin for right wheel
#define I1 8  // Direction pin 1 for left wheel
#define I2 7  // Direction pin 2 for left wheel
#define I4 9  // Direction pin 1 for right wheel
#define I3 12 // Direction pin 2 for right wheel

class movement {
public:
  void move(double targetW, double targetV, double dt, double Lencoder, double Rencoder, double actual_W);
  void startup();
  void calibrateFeedforward();
  void run(int PWM, bool side);

private:
  double LeftPID(double setpoint, double current, double dt);
  double RightPID(double setpoint, double current, double dt);
  double AngularPID(double setpoint, double current, double dt);
  double calculateFeedforward(double targetVelocity, const std::map<double, int>& calibrationMap);

  double integral_angular = 0, prev_err_angular = 0;
  double integral_left = 0, prev_err_left = 0;
  double integral_right = 0, prev_err_right = 0;

  unsigned long macro_timer_L = 0;
  unsigned long macro_timer_R = 0;

  // Acceleration Control
  double current_target_v = 0.0; 

  //PASTE CALIBRATION MAPS BELOW 
  //LEFT MAP:
std::map<double, int> calibrateFeedforward_L = {
  {9.13, 130},
  {10.24, 135},
  {10.67, 140},
  {11.01, 145},
  {11.34, 150},
  {11.54, 155},
  {11.82, 160},
  {12.11, 165},
  {12.32, 170},
  {12.57, 175},
  {12.74, 180},
  {13.01, 185},
  {13.16, 190},
  {13.34, 195},
  {13.52, 200},
  {13.70, 205},
  {13.85, 210},
  {14.03, 215},
  {14.15, 220},
  {14.34, 225},
  {14.46, 230},
  {14.64, 235},
  {14.68, 240},
  {14.84, 245},
  {15.11, 250},
  {15.53, 255},
};

std::map<double, int> calibrateFeedforward_R = {
  {8.29, 130},
  {9.58, 135},
  {10.03, 140},
  {10.40, 145},
  {10.76, 150},
  {11.07, 155},
  {11.34, 160},
  {11.66, 165},
  {11.93, 170},
  {12.21, 175},
  {12.40, 180},
  {12.71, 185},
  {12.90, 190},
  {13.13, 195},
  {13.31, 200},
  {13.52, 205},
  {13.67, 210},
  {13.88, 215},
  {13.99, 220},
  {14.15, 225},
  {14.30, 235},
  {14.31, 230},
  {14.54, 240},
  {14.59, 245},
  {14.79, 250},
  {15.12, 255},
};

};

#endif // MOVEMENT_H