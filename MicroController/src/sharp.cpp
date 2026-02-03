#include "sharp.h"

void read(){
  // Read the sensor output (0-1023, which is 10 bits and fits inside an Arduino int-type)
  front_sharp_val = analogRead(FRONT_SHARP_PIN);
  right_sharp_val = analogRead(RIGHT_SHARP_PIN);
  left_sharp_val = analogRead(LEFT_SHARP_PIN);
}

void print(){
  // Print all values
  Serial.print(front_sharp_val);
  Serial.print(",");
  Serial.print(right_sharp_val);
  Serial.print(",");
  Serial.print(left_sharp_val);
  Serial.print("\n");
}