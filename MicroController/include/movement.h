//movement.h

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "encoders.h"

class movement {    
public:
    void startup();
    void forward(int speed);
    void backward(int speed);
    void stop();
    void turn(int speed_l, int speed_r);
private:
    // Wheel PWM pins (must be a PWM pin)
    static const int EA = 6;
    static const int EB = 9;

    // Wheel direction digital pins
    static const int I1 = 13;
    static const int I2 = 12;
    static const int I3 = 11;
    static const int I4 = 10;
    // Motor PWM command variable [0-255]
    static byte u2;
    static byte u1;
};

#endif // MOVEMENT_H
