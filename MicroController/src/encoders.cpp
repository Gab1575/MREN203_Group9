#include "encoders.h"

// --- Static Member Variable Allocation ---
volatile long encoders::encoder_ticksL = 0;
volatile long encoders::encoder_ticksR = 0;

encoders::encoders(){}

// LEFT This function is called when SIGNAL_A goes HIGH
void encoders::decodeEncoderTicksL() {
  if (digitalRead(SIGNAL_B) == LOW) {
    // SIGNAL_A leads SIGNAL_B, so count one way
    encoder_ticksL--;
  } else {
    // SIGNAL_B leads SIGNAL_A, so count the other way
    encoder_ticksL++;
  }
}

// RIGHT This function is called when SIGNAL_C goes HIGH
void encoders::decodeEncoderTicksR() {
  if (digitalRead(SIGNAL_D) == LOW) {
    // SIGNAL_C leads SIGNAL_D, so count one way
    encoder_ticksR--;
  } else {
    // SIGNAL_D leads SIGNAL_C, so count the other way
    encoder_ticksR++;
  }
}

void encoders::startup() {
  // Set the pin modes for the encoders
  pinMode(SIGNAL_A, INPUT);
  pinMode(SIGNAL_B, INPUT);
  pinMode(SIGNAL_C, INPUT);
  pinMode(SIGNAL_D, INPUT);

  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(SIGNAL_A), encoders::decodeEncoderTicksL, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL_C), encoders::decodeEncoderTicksR, RISING);

  Serial.println("Encoders initialized.");
}

void encoders::run(){
  // Get the elapsed time [ms]
  t_now = millis();

  if (t_now - t_last >= T) {
    // Estimate the rotational speed [rad/s]
    // Calculation: 2*PI * (ticks/TPR) * (1000ms / delta_t)
    omega_L = 2.0 * PI * ((double)encoder_ticksL / (double)TPR) * 1000.0 / (double)(t_now - t_last);
    omega_R = 2.0 * PI * ((double)encoder_ticksR / (double)TPR) * 1000.0 / (double)(t_now - t_last);

    // Record the current time [ms]
    t_last = t_now;

    // Reset the encoder ticks counter
    encoder_ticksL = 0;
    encoder_ticksR = 0;
    }
}

void encoders::print(){
    Serial.print("Left: ");
    Serial.print(omega_L);
    Serial.print(" rad/s | Right: ");
    Serial.print(omega_R);
    Serial.println(" rad/s");
    delay(500);
}