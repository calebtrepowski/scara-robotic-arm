#include <AccelStepper.h>

#include "Joint.hpp"

class ScaraRoboticArm {
public:
  ScaraRoboticArm();
  ~ScaraRoboticArm();

  void goLimitSimultaneous(void);
  void goLimitSequential(void);

private:
  Joint joint_1;
  // Joint joint_2;
  // Joint joint_3;
  // Joint joint_4;
};