#include <Arduino.h>
#include "ScaraRoboticArm.hpp"
#include "Constants.hpp"

ScaraRoboticArm arm;

void setup(void) {
  Serial.begin(115200);
}

void loop(void) {
  Serial.println("Enviar comando para empezar");
  while (Serial.available() == 0) {}
  arm.goLimitSimultaneous();
  delay(200);
  arm.goToAbsoluteArticularPosition(45, -30, 45, 0);
  Serial.print(arm.x);
  Serial.print("\t");
  Serial.print(arm.y);
  Serial.print("\t");
  Serial.println(arm.z);
  while (Serial.available() > 0) { Serial.read(); }
}