#include "movement.h"

//takes in a linear and angular velocity and sends wheel speeds

void movement::startup() {
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
}