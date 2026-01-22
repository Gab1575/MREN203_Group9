#include "movement.h"

//takes in a linear and angular velocity and sends wheel speeds

void movement::startup() {
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
}

void movement::forward(int speed) {
    // Set motor directions for forward movement
    digitalWrite(I1, HIGH);
    digitalWrite(I2, LOW);

    digitalWrite(I3, HIGH);
    digitalWrite(I4, LOW);
    
    // Set motor speeds
    analogWrite(EA, speed);
    analogWrite(EB, speed);
}

void movement::backward(int speed){
    // Set motor directions for backward movement
    digitalWrite(I1, LOW);
    digitalWrite(I2, HIGH);

    digitalWrite(I3, LOW);
    digitalWrite(I4, HIGH);
    
    // Set motor speeds
    analogWrite(EA, speed);
    analogWrite(EB, speed);
}

void movement::stop() {
    // Stop both motors
    analogWrite(EA, 0);
    analogWrite(EB, 0);
}

void movement::turn(int speed_l, int speed_r) {
    // Set motor directions based on speed signs
    if (speed_l >= 0) {
        digitalWrite(I1, HIGH);
        digitalWrite(I2, LOW);
    } else {
        digitalWrite(I1, LOW);
        digitalWrite(I2, HIGH);
        speed_l = -speed_l; // Make speed positive for PWM
    }

    if (speed_r >= 0) {
        digitalWrite(I3, HIGH);
        digitalWrite(I4, LOW);
    } else {
        digitalWrite(I3, LOW);
        digitalWrite(I4, HIGH);
        speed_r = -speed_r; // Make speed positive for PWM
    }
    
    // Set motor speeds
    analogWrite(EA, speed_l);
    analogWrite(EB, speed_r);
}