//encoders.h

#ifndef ENCODERS_H
#define ENCODERS_H

#include <Arduino.h>

#define SIGNAL_A 21
#define SIGNAL_B 1
#define SIGNAL_C 38
#define SIGNAL_D 39

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
    //IRAM_ATTR is needed for interrupt functions, forces code into the faster IRAM memory
    static void IRAM_ATTR decodeEncoderTicksR(); 
    static void IRAM_ATTR decodeEncoderTicksL();

    // Encoder ticks per (motor) revolution (TPR)
    static const int TPR = 3000;

    // Wheel radius [m]
    static constexpr double RHO = 0.0625;    

    // Counter to keep track of encoder ticks
    static volatile long encoder_ticksL;
    static volatile long encoder_ticksR;

    // Sampling interval for measurements in milliseconds
    const unsigned int T = 100; // 0.1 second

    // Counters for milliseconds during interval
    unsigned long t_now = 0;
    unsigned long t_last = 0;
};

#endif // ENCODERS_H