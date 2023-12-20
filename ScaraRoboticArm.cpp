#include "ScaraRoboticArm.hpp"
#include "Constants.hpp"

ScaraRoboticArm::ScaraRoboticArm()
    : joint_1(1,
              JOINT_1_STEP_PIN,
              JOINT_1_DIRECTION_PIN,
              JOINT_1_ANGLE_TO_STEPS,
              JOINT_1_LIMIT_SWITCH_PIN,
              JOINT_1_LIMIT_DIRECTION,
              JOINT_1_LIMIT_POSITION),
      joint_2(2,
              JOINT_2_STEP_PIN,
              JOINT_2_DIRECTION_PIN,
              JOINT_2_DISTANCE_TO_STEPS,
              JOINT_2_LIMIT_SWITCH_PIN,
              JOINT_2_LIMIT_DIRECTION,
              JOINT_2_LIMIT_POSITION),
      joint_3(3,
              JOINT_3_STEP_PIN,
              JOINT_3_DIRECTION_PIN,
              JOINT_3_ANGLE_TO_STEPS,
              JOINT_3_LIMIT_SWITCH_PIN,
              JOINT_3_LIMIT_DIRECTION,
              JOINT_3_LIMIT_POSITION),
      joint_4(4,
              JOINT_4_STEP_PIN,
              JOINT_4_DIRECTION_PIN,
              JOINT_4_ANGLE_TO_STEPS,
              JOINT_4_LIMIT_SWITCH_PIN,
              JOINT_4_LIMIT_DIRECTION,
              JOINT_4_LIMIT_POSITION)
{
  multiStepper.addStepper(joint_1.stepper);
  multiStepper.addStepper(joint_2.stepper);
  multiStepper.addStepper(joint_3.stepper);
  multiStepper.addStepper(joint_4.stepper);
}

ScaraRoboticArm::~ScaraRoboticArm()
{
}

void ScaraRoboticArm::goLimitSimultaneous(void)
{
  byte limitFlags = 0b0000 | (joint_1.isOnLimit() << 0) | (joint_2.isOnLimit() << 1) | (joint_3.isOnLimit() << 2) | (joint_4.isOnLimit() << 3);

  if (limitFlags == 0b1111)
  {
    joint_1.stepper.setCurrentPosition(joint_1.LIMIT_POSITION);
    joint_2.stepper.setCurrentPosition(joint_2.LIMIT_POSITION);
    joint_3.stepper.setCurrentPosition(joint_3.LIMIT_POSITION);
    joint_4.stepper.setCurrentPosition(joint_4.LIMIT_POSITION);
    updateForwardKinematics();
    return;
  }

  joint_1.stepper.setSpeed(500 * joint_1.LIMIT_DIRECTION);
  joint_2.stepper.setSpeed(750 * joint_2.LIMIT_DIRECTION);
  joint_3.stepper.setSpeed(500 * joint_3.LIMIT_DIRECTION);
  joint_4.stepper.setSpeed(250 * joint_4.LIMIT_DIRECTION);

  while (limitFlags != 0b1111)
  {
    if (!(limitFlags & 0b0001))
    {
      joint_1.stepper.runSpeed();
      if (joint_1.isOnLimit())
      {
        limitFlags |= 0b0001;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_1.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b0010))
    {
      joint_2.stepper.runSpeed();
      if (joint_2.isOnLimit())
      {
        limitFlags |= 0b0010;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_2.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b0100))
    {
      joint_3.stepper.runSpeed();
      if (joint_3.isOnLimit())
      {
        limitFlags |= 0b0100;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_3.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b1000))
    {
      joint_4.stepper.runSpeed();
      if (joint_4.isOnLimit())
      {
        limitFlags |= 0b1000;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_4.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }
  }

  joint_1.stepper.setCurrentPosition(joint_1.LIMIT_POSITION);
  joint_2.stepper.setCurrentPosition(joint_2.LIMIT_POSITION);
  joint_3.stepper.setCurrentPosition(joint_3.LIMIT_POSITION);
  joint_4.stepper.setCurrentPosition(joint_4.LIMIT_POSITION);
  updateForwardKinematics();

  joint_1.stepper.setSpeed(0);
  joint_2.stepper.setSpeed(0);
  joint_3.stepper.setSpeed(0);
  joint_4.stepper.setSpeed(0);
}

void ScaraRoboticArm::goLimitSequential(void)
{
  joint_4.goLimit(250);
  joint_3.goLimit(500);
  joint_2.goLimit(1000);
  joint_1.goLimit(500);
  joint_1.stepper.setCurrentPosition(-2000);
  joint_2.stepper.setCurrentPosition(0);
  joint_3.stepper.setCurrentPosition(-2710);
  updateForwardKinematics();
}

void ScaraRoboticArm::goToAbsoluteArticularPosition(const int joint_1_angle,
                                                    const int joint_2_distance,
                                                    const int joint_3_angle,
                                                    const int joint_4_angle)
{
  // TODO: verify angles and distance inside valid ranges
  targetPositions[0] = joint_1.calculateMovementSteps(joint_1_angle);
  targetPositions[1] = joint_2.calculateMovementSteps(joint_2_distance);
  targetPositions[2] = joint_3.calculateMovementSteps(joint_3_angle);
  targetPositions[3] = joint_4.calculateMovementSteps(joint_4_angle);
  multiStepper.moveTo(targetPositions);
  multiStepper.runSpeedToPosition();
  updateForwardKinematics();
}

void ScaraRoboticArm::updateForwardKinematics(void)
{
  float theta1 = joint_1.stepper.currentPosition() / joint_1.UNIT_TO_STEPS;
  Serial.println(theta1);
  float theta1F = (theta1)*PI / 180;
  float theta3 = joint_3.stepper.currentPosition() / joint_3.UNIT_TO_STEPS;
  Serial.println(theta3);
  float theta3F = (theta3)*PI / 180;
  x = round(L1 * cos(theta1F) + L2 * cos(theta1F + theta3F));
  y = round(L1 * sin(theta1F) + L2 * sin(theta1F + theta3F));
  z = joint_2.stepper.currentPosition() / joint_2.UNIT_TO_STEPS;
}