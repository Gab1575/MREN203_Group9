//movement.h

#ifndef MOVEMENT_H
#define MOVEMENT_H

#include <Arduino.h>
#include "encoders.h"

class movement {    
public:
    void startup();
    
private:
    // Wheel PWM pins (must be a PWM pin)
    static const int EA = 6;
    static const int EB = 5;

    // Wheel direction digital pins
    static const int I1 = 7;
    static const int I2 = 4;
    static const int I3 = 8;
    static const int I4 = 9;
    // Motor PWM command variable [0-255]
    static byte u2;
    static byte u1;
};

#endif // MOVEMENT_H
