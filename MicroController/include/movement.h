    //movement.h

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "encoders.h"
#include "IMU.h"

//PID CONTROLS:
#define LIN_KP 1.0
#define LIN_KI 0.0
#define LIN_KD 0.0
#define LIN_MAX_INTEGRAL 100.0

#define ANG_KP 1.0
#define ANG_KI 0.0
#define ANG_KD 0.0 
#define ANG_MAX_INTEGRAL 100.0


#define EA 6  // PWM pin for left wheel
#define EB 11  // PWM pin for right wheel

#define I1 8 // Direction pin 1 for left wheel
#define I2 7 // Direction pin 2 for left wheel
#define I4 9 // Direction pin 1 for right wheel
#define I3 12 // Direction pin 2 for right wheel

class movement {    
public:
    void move(double targetW, double targetV, double dt);
    void startup();
    void forward(int speed);
    void backward(int speed);
    void stop();
    void turn(int speed_l, int speed_r);
private:
    double LinearPID(double setpoint, double current, double dt);
    double AngularPID(double setpoint, double current, double dt);

    InternalIMU imu;
    encoders enc;
    double integral_linear = 0, prev_err_linear = 0;
    double integral_angular = 0, prev_err_angular = 0;
};

#endif // MOVEMENT_H