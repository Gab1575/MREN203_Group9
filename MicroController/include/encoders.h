//encoders.h

#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

class encoders {
public:
    encoders();
    void startup();
    void run();
    void print();
    // Variable to store estimated angular rate of left wheel [rad/s]
    double omega_L = 0.0;
    double omega_R = 0.0;

private:
    static void decodeEncoderTicksR();
    static void decodeEncoderTicksL();

    static const byte SIGNAL_A = 2;
    static const byte SIGNAL_B = 3;
    static const byte SIGNAL_C = 4;
    static const byte SIGNAL_D = 5;

    // Encoder ticks per (motor) revolution (TPR)
    static const int TPR = 3000;

    // Wheel radius [m]
    static const double RHO = 0.0625;
    
    // Counter to keep track of encoder ticks
    static volatile long encoder_ticksL;
    static volatile long encoder_ticksR;

    // Sampling interval for measurements in milliseconds
    const unsigned int T = 1000; // 1 second

    // Counters for milliseconds during interval
    unsigned long t_now = 0;
    unsigned long t_last = 0;
};

#endif // ENCODERS_H
