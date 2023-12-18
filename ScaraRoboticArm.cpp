#include "ScaraRoboticArm.hpp"
#include "Constants.hpp"

ScaraRoboticArm::ScaraRoboticArm()
  : joint_1(1,
            JOINT_1_STEP_PIN,
            JOINT_1_DIRECTION_PIN,
            JOINT_1_ANGLE_TO_STEPS,
            JOINT_1_LIMIT_SWITCH_PIN,
            JOINT_1_LIMIT_DIRECTION) {
}

ScaraRoboticArm::~ScaraRoboticArm() {}

void ScaraRoboticArm::goLimitSimultaneous(void) {
  byte limitFlags = 0b0000;

  limitFlags |= (joint_1.isOnLimit() << 0);
  // limitFlags |= (joint_2.isOnLimit() << 1);
  // limitFlags |= (joint_3.isOnLimit() << 2);
  // limitFlags |= (joint_4.isOnLimit() << 3);

  if (limitFlags == 0b1111) { return; }

  joint_1.stepper.setSpeed(-500);
  // joint_2.stepper.setSpeed(-500);
  // joint_3.stepper.setSpeed(500);
  // joint_4.stepper.setSpeed(250);

  while (limitFlags != 0b1111) {
    if (!(limitFlags & 0b0001)) {
      joint_1.stepper.runSpeed();
      limitFlags |= joint_1.isOnLimit() << 0;
    }

    // if (!(limitFlags & 0b0010)) {
    //   joint_2.stepper.runSpeed();
    //   limitFlags |= joint_2.isOnLimit() << 1;
    // }

    // if (!(limitFlags & 0b0100)) {
    //   joint_3.stepper.runSpeed();
    //   limitFlags |= joint_3.isOnLimit() << 2;
    // }

    // if (!(limitFlags & 0b1000)) {
    //   joint_4.stepper.runSpeed();
    //   limitFlags |= joint_4.isOnLimit() << 3;
    // }
  }
}

void ScaraRoboticArm::goLimitSequential(void) {
  joint_1.goLimit(500);
  // joint_2.goLimit(500);
  // joint_3.goLimit(500);
  // joint_4.goLimit(250);
}