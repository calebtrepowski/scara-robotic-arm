#include "Joint.hpp"
#include "Constants.hpp"

Joint::Joint(const byte jointNumber,
             const byte stepPin,
             const byte directionPin,
             const float unitToSteps,
             const byte limitSwitchPin,
             const char limitDirection,
             const long limitPosition)
  : JOINT_NUMBER(jointNumber),
    stepper(AccelStepper::DRIVER, stepPin, directionPin),
    UNIT_TO_STEPS(unitToSteps),
    LIMIT_SWITCH_PIN(limitSwitchPin),
    LIMIT_DIRECTION(limitDirection),
    LIMIT_POSITION(limitPosition) {
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(500);
}

Joint::~Joint() {}

void Joint::goLimit(const int speed) {
  DEBUG_PRINT("Joint ");
  DEBUG_PRINT(JOINT_NUMBER);
  DEBUG_PRINT(" going limit with speed ");
  DEBUG_PRINTLN(LIMIT_DIRECTION * abs(speed));

  if (isOnLimit()) {
    DEBUG_PRINT("Joint ");
    DEBUG_PRINT(JOINT_NUMBER);
    DEBUG_PRINTLN(" already on limit.");
    return;
  }

  stepper.setSpeed(LIMIT_DIRECTION * abs(speed));

  while (!isOnLimit()) {
    stepper.runSpeed();
  }

  DEBUG_PRINT("Joint ");
  DEBUG_PRINT(JOINT_NUMBER);
  DEBUG_PRINTLN(" on limit!");

  Serial.println(LIMIT_POSITION);

  stepper.setCurrentPosition(LIMIT_POSITION);
  stepper.setSpeed(0);
}

bool Joint::isOnLimit(void) {
  return digitalRead(LIMIT_SWITCH_PIN);
}

long Joint::calculateMovementSteps(const int units) {
  return (long)(units)*UNIT_TO_STEPS;
}
