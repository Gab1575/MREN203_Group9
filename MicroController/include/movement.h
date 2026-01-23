//movement.h

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "encoders.h"
#include "IMU.h"

#define EA 6  // PWM pin for left wheel
#define EB 9  // PWM pin for right wheel

#define I1 13 // Direction pin 1 for left wheel
#define I2 12 // Direction pin 2 for left wheel
#define I3 11 // Direction pin 1 for right wheel
#define I4 10 // Direction pin 2 for right wheel

class movement {    
public:
    void startup();
    void forward(int speed);
    void backward(int speed);
    void stop();
    void turn(int speed_l, int speed_r);
private:

};

#endif // MOVEMENT_H