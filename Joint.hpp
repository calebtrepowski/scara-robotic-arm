#include <Arduino.h>
#include <AccelStepper.h>

class Joint {
public:
  Joint(const byte jointNumber,
        const byte stepPin,
        const byte directionPin,
        const float unitToSteps,
        const byte limitSwitchPin,
        const char limitDirection,
        const long limitPosition);
  ~Joint();

  void goLimit(const int speed);
  bool isOnLimit(void);
  long calculateMovementSteps(const float units);
  double getArticularPosition(void);
  void setArticularPosition(double articularPositionUnits);

  friend class ScaraRoboticArm;
  friend void setup(void);
  friend void loop(void);

private:
  const byte JOINT_NUMBER;
  AccelStepper stepper;
  const float UNIT_TO_STEPS;
  const byte LIMIT_SWITCH_PIN;
  const char LIMIT_DIRECTION;
  const long LIMIT_POSITION;
};