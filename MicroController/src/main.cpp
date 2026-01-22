//main.cpp
#include <Arduino.h>
#include "encoders.h"
#include "movement.h"

double Lspeed;
double Rspeed; 

encoders Encoders;
movement Movement;

void setup() {
  Serial.begin(9600);
  Encoders.startup();
  Movement.startup();
}
void loop() {
  Movement.forward(200);
  Encoders.run();
  Encoders.print();
  delay(100);
}