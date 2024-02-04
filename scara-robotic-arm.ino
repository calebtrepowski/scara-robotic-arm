#include <Arduino.h>
#include <Servo.h>
#include "ScaraRoboticArm.hpp"
#include <GCodeParser.h>
#include "Constants.hpp"

#define SEND_STATUS(...) \
  Serial.print("$["); \
  Serial.print(__VA_ARGS__); \
  Serial.print("]");

GCodeParser GCode = GCodeParser();
ScaraRoboticArm arm;

enum {
  READY = 0,
  MOVEMENT_FAILED = 1,
} StatusCode;


void setup(void) {
  Serial.begin(115200);
  arm.gripperServo.setup(A0, 600, 2500);
  SEND_STATUS(READY);
  Serial.println();
}

void loop(void) {
  // Serial.print(arm.joint_1.isOnLimit());
  // Serial.print(" ");
  // Serial.print(arm.joint_2.isOnLimit());
  // Serial.print(" ");
  // Serial.print(arm.joint_3.isOnLimit());
  // Serial.print(" ");
  // Serial.println(arm.joint_4.isOnLimit());
  // delay(1000);
  if (Serial.available() > 0) {
    if (GCode.AddCharToLine(Serial.read())) {
      GCode.ParseLine();
      GCode.RemoveCommentSeparators();
      if (GCode.HasWord('G')) {
        // Serial.print("Process G code: ");
        // Serial.println(GCode.line);
        unsigned int gCodeFunction = GCode.GetWordValue('G');
        // Serial.println(gCodeFunction);
        switch (gCodeFunction) {
          case 0:
            {
              double x = arm.x, y = arm.y, z = arm.z, t = arm.joint_4.getArticularPosition();
              if (GCode.HasWord('X')) { x = GCode.GetWordValue('X'); }
              if (GCode.HasWord('Y')) { y = GCode.GetWordValue('Y'); }
              if (GCode.HasWord('Z')) { z = GCode.GetWordValue('Z'); }
              if (GCode.HasWord('T')) { z = GCode.GetWordValue('T'); }
              bool status = arm.goToCartesianPosition(x, y, z);
              SEND_STATUS(status ? READY : MOVEMENT_FAILED);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 1:
            {
              double x = arm.x, y = arm.y, z = arm.z;
              if (GCode.HasWord('X')) { x = GCode.GetWordValue('X'); }
              if (GCode.HasWord('Y')) { y = GCode.GetWordValue('Y'); }
              if (GCode.HasWord('Z')) { z = GCode.GetWordValue('Z'); }
              arm.interpolateLine(x, y, z);
              SEND_STATUS(READY);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 10:
            {
              arm.joint_2.goLimit(500);
              arm.goLimitSimultaneous();
              SEND_STATUS(READY);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 11:
            {
              double h = arm.joint_1.getArticularPosition(),
                     j = arm.joint_2.getArticularPosition(),
                     k = arm.joint_3.getArticularPosition(),
                     l = arm.joint_4.getArticularPosition();
              if (GCode.HasWord('H')) { h = GCode.GetWordValue('H'); }
              if (GCode.HasWord('J')) { j = GCode.GetWordValue('J'); }
              if (GCode.HasWord('K')) { k = GCode.GetWordValue('K'); }
              if (GCode.HasWord('L')) { l = GCode.GetWordValue('L'); }
              bool status = arm.goToAbsoluteArticularPosition(h, j, k, l);
              SEND_STATUS(status ? READY : MOVEMENT_FAILED);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 12:
            {
              double h = arm.joint_1.getArticularPosition(),
                     j = arm.joint_2.getArticularPosition(),
                     k = arm.joint_3.getArticularPosition(),
                     l = arm.joint_4.getArticularPosition();
              if (GCode.HasWord('H')) { h = GCode.GetWordValue('H'); }
              if (GCode.HasWord('J')) { j = GCode.GetWordValue('J'); }
              if (GCode.HasWord('K')) { k = GCode.GetWordValue('K'); }
              if (GCode.HasWord('L')) { l = GCode.GetWordValue('L'); }
              arm.joint_1.setArticularPosition(h);
              arm.joint_2.setArticularPosition(j);
              arm.joint_3.setArticularPosition(k);
              arm.joint_4.setArticularPosition(l);
              arm.updateForwardKinematics();
              SEND_STATUS(READY);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 20:
            {
              byte P = 255;
              if (GCode.HasWord('P')) { P = GCode.GetWordValue('P'); }
              arm.gripperServo.grab(P);
              SEND_STATUS(READY);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          case 21:
            {
              arm.gripperServo.release();
              SEND_STATUS(READY);
              Serial.print("XYZ:(");
              Serial.print(arm.x);
              Serial.print(",");
              Serial.print(arm.y);
              Serial.print(",");
              Serial.print(arm.z);
              Serial.print(");J1J2J3J4:(");
              Serial.print(arm.joint_1.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_2.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_3.getArticularPosition());
              Serial.print(",");
              Serial.print(arm.joint_4.getArticularPosition());
              Serial.println(")&");
              break;
            }
          default:
            break;
        }
      }
    }
  }
}
