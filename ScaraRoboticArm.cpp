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
            JOINT_4_LIMIT_POSITION) {
  multiStepper.addStepper(joint_1.stepper);
  multiStepper.addStepper(joint_2.stepper);
  multiStepper.addStepper(joint_3.stepper);
  multiStepper.addStepper(joint_4.stepper);
}

ScaraRoboticArm::~ScaraRoboticArm() {
}

void ScaraRoboticArm::goLimitSimultaneous(void) {
  byte limitFlags = 0b0000 | (joint_1.isOnLimit() << 0) | (joint_2.isOnLimit() << 1) | (joint_3.isOnLimit() << 2) | (joint_4.isOnLimit() << 3);

  if (limitFlags == 0b1111) {
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

  while (limitFlags != 0b1111) {
    if (!(limitFlags & 0b0001)) {
      joint_1.stepper.runSpeed();
      if (joint_1.isOnLimit()) {
        limitFlags |= 0b0001;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_1.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b0010)) {
      joint_2.stepper.runSpeed();
      if (joint_2.isOnLimit()) {
        limitFlags |= 0b0010;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_2.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b0100)) {
      joint_3.stepper.runSpeed();
      if (joint_3.isOnLimit()) {
        limitFlags |= 0b0100;
        DEBUG_PRINT("Joint ");
        DEBUG_PRINT(joint_3.JOINT_NUMBER);
        DEBUG_PRINTLN(" on limit!");
      }
    }

    if (!(limitFlags & 0b1000)) {
      joint_4.stepper.runSpeed();
      if (joint_4.isOnLimit()) {
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

void ScaraRoboticArm::goLimitSequential(void) {
  joint_4.goLimit(250);
  joint_3.goLimit(500);
  joint_2.goLimit(1000);
  joint_1.goLimit(500);
  joint_1.stepper.setCurrentPosition(-2000);
  joint_2.stepper.setCurrentPosition(0);
  joint_3.stepper.setCurrentPosition(-2710);
  updateForwardKinematics();
}

void ScaraRoboticArm::goToAbsoluteArticularPosition(const float joint_1_angle,
                                                    const float joint_2_distance,
                                                    const float joint_3_angle,
                                                    const float joint_4_angle) {
  // TODO: verify angles and distance inside valid ranges
  if (joint_1_angle < -40 || joint_1_angle > 250) {
    Serial.print("No se puede: joint 1 angle: ");
    Serial.println(joint_1_angle);
    return;
  }
  if (joint_2_distance > 0 || joint_2_distance < -200) {
    Serial.print("No se puede: joint 2 distance: ");
    Serial.println(joint_2_distance);
    return;
  }
  if (joint_3_angle < -150 || joint_3_angle > 150) {
    Serial.print("No se puede: joint 3 angle: ");
    Serial.println(joint_3_angle);
    return;
  }
  if (joint_4_angle < 0 || joint_4_angle > 340) {
    Serial.print("No se puede: joint 4 angle: ");
    Serial.println(joint_4_angle);
    return;
  }
  targetPositions[0] = joint_1.calculateMovementSteps(joint_1_angle);
  targetPositions[1] = joint_2.calculateMovementSteps(joint_2_distance);
  targetPositions[2] = joint_3.calculateMovementSteps(joint_3_angle);
  targetPositions[3] = joint_4.calculateMovementSteps(joint_4_angle);

  multiStepper.moveTo(targetPositions);
  multiStepper.runSpeedToPosition();
  updateForwardKinematics();
}

void ScaraRoboticArm::updateForwardKinematics(void) {
  float theta1 = joint_1.stepper.currentPosition() / joint_1.UNIT_TO_STEPS;
  float theta1F = (theta1)*PI / 180;
  float theta3 = joint_3.stepper.currentPosition() / joint_3.UNIT_TO_STEPS;
  float theta3F = (theta3)*PI / 180;
  x = L1 * cos(theta1F) + L2 * cos(theta1F + theta3F);
  y = L1 * sin(theta1F) + L2 * sin(theta1F + theta3F);
  z = joint_2.stepper.currentPosition() / joint_2.UNIT_TO_STEPS;
}

ArticularCoordinate ScaraRoboticArm::calculateInverseKinematics(const double targetX, const double targetY, const double targetZ) {
  double xx = targetX * targetX;
  double yy = targetY * targetY;
  double L1L1 = L1 * L1;
  double L2L2 = L2 * L2;
  double L1L2 = L1 * L2;
  double numerador = xx + yy - L1L1 - L2L2;
  double denominador = 2 * L1L2;
  double theta3R = acos(numerador / denominador);  // 0 a 180
  double theta3D = theta3R * 180 / PI;

  numerador = L2 * sin(theta3R);
  denominador = sqrt(xx + yy);
  double M = asin(numerador / denominador);
  double theta1R = atan(targetY / targetX) - M;
  double theta1D = theta1R * 180 / PI;  // -90 a +90


  if (targetX < 0 && targetY > 0) {
    theta1D += 180;
  }

  return {
    .h = theta1D, .j = targetZ, .k = theta3D, .l = 0
  };
}


void ScaraRoboticArm::goToCartesianPosition(const double targetX, const double targetY, const double targetZ) {
  double xx = targetX * targetX;
  double yy = targetY * targetY;
  if (sqrt(xx + yy) > L1 + L2) {
    Serial.println("no se puede, distancia maxima superada");
    return;
  }
  // double L1L1 = L1 * L1;
  // double L2L2 = L2 * L2;
  // double L1L2 = L1 * L2;
  // double numerador = xx + yy - L1L1 - L2L2;
  // double denominador = 2 * L1L2;
  // double theta3R = acos(numerador / denominador);  // 0 a 180
  // double theta3D = theta3R * 180 / PI;

  // numerador = L2 * sin(theta3R);
  // denominador = sqrt(xx + yy);
  // double M = asin(numerador / denominador);
  // double theta1R = atan(targetY / targetX) - M;
  // double theta1D = theta1R * 180 / PI;  // -90 a +90

  // if (targetX < 0 && targetY > 0) {
  //   theta1D += 180;
  // }

  ArticularCoordinate ap = calculateInverseKinematics(targetX, targetY, targetZ);

  if (ap.h < -45 || ap.h > 345) {
    Serial.print("no se puede, theta1D: ");
    Serial.println(ap.h);
    return;
  }
  if (ap.k < -45 || ap.k > 345) {
    Serial.print("no se puede, theta3D: ");
    Serial.println(ap.k);
    return;
  }

  Serial.print(ap.h);
  Serial.print("\t");
  Serial.println(ap.k);

  // if (targetX >= 0 & targetY >= 0) {
  //   theta1D = 90 - theta1D;
  // }
  // if (targetX < 0 & targetY < 0) {
  //   theta1D = 270 - theta1D;
  //   // phi = 270 - theta1D - theta3D;
  //   // phi = (-1) * phi;
  // }
  // if (targetX > 0 & targetY < 0) {
  //   theta1D = -90 - theta1D;
  // }
  // if (targetX < 0 & targetY == 0) {
  //   theta1D = 270 + theta1D;
  // }
  goToAbsoluteArticularPosition(ap.h, ap.j, ap.k, 0);
}

void ScaraRoboticArm::interpolateLine(const double targetX, const double targetY, const double targetZ, byte interpolationSegments = 10) {
  double deltaX = (targetX - x) / interpolationSegments;
  double deltaY = (targetY - y) / interpolationSegments;
  double deltaZ = (targetZ - z) / interpolationSegments;
  ArticularCoordinate targetCoordinate = calculateInverseKinematics(targetX, targetY, targetZ);
  if (targetCoordinate.h > 345 || targetCoordinate.h < 0) {
    Serial.println("no se puede, pasa h");
    return;
  }
  for (byte i = 0; i < interpolationSegments - 1; ++i) {
    goToCartesianPosition(x + deltaX, y + deltaY, z + deltaZ);
  }
  goToCartesianPosition(targetX, targetY, targetZ);
}
