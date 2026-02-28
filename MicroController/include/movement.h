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
#define MACRO_WINDOW_MS 100 

// PID CONTROLS:
#define LEFT_KP 0
#define LEFT_KD 0
#define LEFT_KI 0
#define LEFT_MAX_INTEGRAL 100.0

#define RIGHT_KP 0
#define RIGHT_KD 0
#define RIGHT_KI 0
#define RIGHT_MAX_INTEGRAL 100.0

#define ANG_KP 0
#define ANG_KI 0
#define ANG_KD 0.0
#define ANG_MAX_INTEGRAL 0

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
  void forward(int speed);
  void backward(int speed);
  void stop();
  void turn(int speed_l, int speed_r);
  void calibrateFeedforward();

private:
  double AngularPID(double setpoint, double current, double dt);
  double LeftPID(double setpoint, double current, double dt);
  double RightPID(double setpoint, double current, double dt);
  double calculateFeedforward(double targetVelocity, const std::map<double, int>& calibrationMap);

  double integral_angular = 0, prev_err_angular = 0;
  double integral_left = 0, prev_err_left = 0;
  double integral_right = 0, prev_err_right = 0;
  
  bool is_pulsing_off_left = false; 
  bool is_pulsing_off_right = false;
  //PASTE CALIBRATION MAPS BELOW 
  //LEFT MAP:
  std::map<double, int> calibrateFeedforward_L;

  //RIGHT MAP:
  std::map<double, int> calibrateFeedforward_R;
};

#endif // MOVEMENT_H