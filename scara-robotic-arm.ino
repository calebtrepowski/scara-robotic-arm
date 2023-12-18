#include <Arduino.h>
#include "ScaraRoboticArm.hpp"

ScaraRoboticArm arm;

void setup(void) {
  Serial.begin(9600);
  delay(500);
  Serial.println("Enviar comando para empezar: ");
  while (Serial.available() == 0) {}
  arm.goLimitSequential();
}

void loop(void) {
}