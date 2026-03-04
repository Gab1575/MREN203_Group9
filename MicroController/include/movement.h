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
#define MACRO_WINDOW_MS 25
// PID CONTROLS:
#define LEFT_KP 2.5
#define LEFT_KD 0.5
#define LEFT_KI 5
#define LEFT_MAX_INTEGRAL 75

#define RIGHT_KP 2.5
#define RIGHT_KD 0.5
#define RIGHT_KI 5
#define RIGHT_MAX_INTEGRAL 75

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


// ==========================================
// COPY AND PASTE CALIBRATION MAPS BELOW
// ==========================================

std::map<double, int> calibrateFeedforward_L = {
  {-0.06, 10},
  {-0.06, 15},
  {0.00, 5},
  {0.04, 30},
  {0.04, 25},
  {0.07, 20},
  {2.83, 35},
  {3.67, 45},
  {3.69, 40},
  {4.04, 55},
  {4.04, 50},
  {4.35, 60},
  {4.64, 65},
  {4.65, 70},
  {4.95, 75},
  {5.26, 85},
  {5.26, 80},
  {5.55, 95},
  {5.55, 90},
  {5.92, 100},
  {6.37, 105},
  {6.38, 110},
  {6.85, 115},
  {7.35, 120},
  {7.37, 125},
  {10.46, 130},
  {11.92, 135},
  {12.32, 140},
  {12.67, 145},
  {12.98, 150},
  {13.29, 155},
  {13.57, 160},
  {13.83, 165},
  {14.05, 170},
  {14.28, 175},
  {14.51, 180},
  {14.70, 185},
  {14.87, 190},
  {15.02, 195},
  {15.12, 205},
  {15.12, 200},
  {15.35, 215},
  {15.35, 210},
  {15.51, 220},
  {15.53, 225},
  {15.57, 230},
  {15.61, 235},
  {15.66, 240},
  {15.72, 245},
  {15.97, 250},
  {16.32, 255},
};

std::map<double, int> calibrateFeedforward_R = {
  {-0.03, 15},
  {-0.02, 10},
  {0.04, 20},
  {0.05, 5},
  {2.62, 25},
  {3.48, 30},
  {3.86, 35},
  {4.35, 45},
  {4.37, 40},
  {4.74, 55},
  {4.75, 50},
  {5.12, 60},
  {5.58, 70},
  {5.58, 65},
  {5.90, 75},
  {6.36, 80},
  {6.37, 85},
  {6.76, 95},
  {6.83, 90},
  {7.27, 100},
  {7.68, 105},
  {7.72, 110},
  {8.13, 115},
  {8.54, 120},
  {8.56, 125},
  {11.11, 130},
  {12.11, 135},
  {12.42, 140},
  {12.70, 145},
  {13.01, 150},
  {13.29, 155},
  {13.56, 160},
  {13.85, 165},
  {14.06, 170},
  {14.22, 175},
  {14.30, 180},
  {14.32, 185},
  {14.67, 190},
  {14.67, 195},
  {14.78, 200},
  {14.81, 205},
  {14.82, 215},
  {14.96, 210},
  {15.01, 225},
  {15.09, 230},
  {15.14, 220},
  {15.27, 235},
  {15.37, 240},
  {15.55, 245},
  {15.65, 250},
  {15.75, 255},
};


};

#endif // MOVEMENT_H