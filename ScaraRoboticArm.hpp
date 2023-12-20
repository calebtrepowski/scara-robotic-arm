#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Joint.hpp"

class ScaraRoboticArm {
public:
  ScaraRoboticArm();
  ~ScaraRoboticArm();

  void goLimitSimultaneous(void);
  void goLimitSequential(void);
  void goToAbsoluteArticularPosition(const int joint_1_angle,
                                     const int joint_2_distance,
                                     const int joint_3_angle,
                                     const int joint_4_angle);
  void updateForwardKinematics(void);

  friend void setup(void);
  friend void loop(void);

private:
  Joint joint_1;
  Joint joint_2;
  Joint joint_3;
  Joint joint_4;
  MultiStepper multiStepper;

  float x, y, z;

  long targetPositions[4];
};