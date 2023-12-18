#include "Arduino.h"
#include "Joint.hpp"

Joint::Joint(const byte jointNumber,
             const byte stepPin,
             const byte directionPin,
             const float unitToSteps,
             const byte limitSwitchPin,
             const byte limitDirection)
  : JOINT_NUMBER(jointNumber),
    stepper(AccelStepper::DRIVER, stepPin, directionPin),
    UNIT_TO_STEPS(unitToSteps),
    LIMIT_SWITCH_PIN(limitSwitchPin),
    LIMIT_DIRECTION(limitDirection) {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  stepper.setMaxSpeed(4000);
  stepper.setAcceleration(2000);
}

Joint::~Joint() {}

void Joint::goLimit(const int speed) {
  if (isOnLimit()) { return; }

  stepper.setSpeed(LIMIT_DIRECTION * abs(speed));

  while (!isOnLimit()) {
    stepper.runSpeed();
  }

  stepper.setSpeed(0);
}

bool Joint::isOnLimit(void) {
  return digitalRead(LIMIT_SWITCH_PIN);
}