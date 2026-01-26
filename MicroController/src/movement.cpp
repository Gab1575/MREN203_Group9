#include "movement.h"

void movement::startup() {
    pinMode(EA, OUTPUT);
    pinMode(I1, OUTPUT);
    pinMode(I2, OUTPUT);
    pinMode(EB, OUTPUT);
    pinMode(I3, OUTPUT);    
    pinMode(I4, OUTPUT);
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
double movement::LinearPID(double setpoint, double current, double dt) {
    double error = setpoint - current;
    integral_linear += error * dt;
    if (integral_linear > LIN_MAX_INTEGRAL) integral_linear = LIN_MAX_INTEGRAL;     //windup guard
    if (integral_linear < -LIN_MAX_INTEGRAL) integral_linear = -LIN_MAX_INTEGRAL;
    double derivative = (error - prev_err_linear) / dt;
    prev_err_linear = error;
    return (LIN_KP * error) + (LIN_KI * integral_linear) + (LIN_KD * derivative);
}

double movement::AngularPID(double setpoint, double current, double dt) {
    double error = setpoint - current;
    integral_angular += error * dt;
    if (integral_angular > ANG_MAX_INTEGRAL) integral_angular = ANG_MAX_INTEGRAL;   //windup guard
    if (integral_angular < -ANG_MAX_INTEGRAL) integral_angular = -ANG_MAX_INTEGRAL;
    double derivative = (error - prev_err_angular) / dt;
    prev_err_angular = error;
    return (ANG_KP * error) + (ANG_KI * integral_angular) + (ANG_KD * derivative);
}

void movement::move(double targetW, double targetV, double dt) {

    IMU::IMUData data = imu.read();
    double W = data.gyroZ; //anguklar velocity from IMU
    double V = (enc.omega_L + enc.omega_R) / 2.0; //linear velocity from encoders

    double v_out = LinearPID(targetV, V, dt);
    double w_out = AngularPID(targetW, W, dt);

    // Left = Linear - Angular | Right = Linear + Angular
    double left = v_out - w_out;
    double right = v_out + w_out;

    if (left > 255) left = 255;         //not sure if this is the right way to clamp with 
    else if (left < -255) left = -255;
   
    if (right > 255) right = 255;
    else if (right < -255) right = -255;

    turn(static_cast<int>(left), static_cast<int>(right)); //converts to int for motor commands
}