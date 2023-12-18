#include <Arduino.h>
#include <AccelStepper.h>

class Joint {
public:
  Joint(const byte jointNumber,
        const byte stepPin,
        const byte directionPin,
        const float unitToSteps,
        const byte limitSwitchPin,
        const byte limitDirection);
  ~Joint();

  void goLimit(int speed);
  bool isOnLimit(void);

  friend class ScaraRoboticArm;

private:
  const byte JOINT_NUMBER;
  AccelStepper stepper;
  const float UNIT_TO_STEPS;
  const byte LIMIT_SWITCH_PIN;
  const byte LIMIT_DIRECTION;
};