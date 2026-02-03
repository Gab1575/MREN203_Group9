//sharp.h

#ifndef SHARP_H
#define SHARP_H

#include <Arduino.h>

class Sharp {
public:

int front_sharp_val = 0;   
int right_sharp_val = 0;
int left_sharp_val = 0;

void read();
void print();
   
private:
  
const byte FRONT_SHARP_PIN = A2;
const byte RIGHT_SHARP_PIN = A1;
const byte LEFT_SHARP_PIN = A0;

};

#endif // SHARP_H
