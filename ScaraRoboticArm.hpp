#include <AccelStepper.h>
#include <MultiStepper.h>
#include "Joint.hpp"
#include "GripperServo.hpp"

struct CartesianCoordinate {
  double x, y, z;
};

struct ArticularCoordinate {
  double h, j, k, l;
};

class ScaraRoboticArm {
public:
  ScaraRoboticArm();
  ~ScaraRoboticArm();

  void goLimitSimultaneous(void);
  void goLimitSequential(void);
  void goToAbsoluteArticularPosition(const float joint_1_angle,
                                     const float joint_2_distance,
                                     const float joint_3_angle,
                                     const float joint_4_angle);
  void updateForwardKinematics(void);
  ArticularCoordinate calculateInverseKinematics(const double targetX, const double targetY, const double targetZ);
  void goToCartesianPosition(const double targetX, const double targetY, const double targetZ);
  void interpolateLine(const double targetX, const double targetY, const double targetZ, byte interpolationSegments = 10);

  friend void setup(void);
  friend void loop(void);

private:
  Joint joint_1;
  Joint joint_2;
  Joint joint_3;
  Joint joint_4;
  MultiStepper multiStepper;

  GripperServo gripperServo;

  double x, y, z, phi;



  long targetPositions[4];
};