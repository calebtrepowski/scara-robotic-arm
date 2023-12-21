#include <Arduino.h>
#include <Servo.h>
#include "ScaraRoboticArm.hpp"
#include <GCodeParser.h>
#include "Constants.hpp"

GCodeParser GCode = GCodeParser();
ScaraRoboticArm arm;

void setup(void) {
  Serial.begin(115200);
  arm.gripperServo.setup(A0, 600, 2500);
  Serial.println("Enviar codigos G: ");
}

void loop(void) {
  if (Serial.available() > 0) {
    if (GCode.AddCharToLine(Serial.read())) {
      GCode.ParseLine();
      GCode.RemoveCommentSeparators();
      if (GCode.HasWord('G')) {
        Serial.print("Process G code: ");
        Serial.println(GCode.line);
        unsigned int gCodeFunction = GCode.GetWordValue('G');
        Serial.println(gCodeFunction);
        switch (gCodeFunction) {
          case 0:
            {
              double x = arm.x, y = arm.y, z = arm.z;
              Serial.print(x);
              Serial.print("\t");
              Serial.print(y);
              Serial.print("\t");
              Serial.println(z);
              if (GCode.HasWord('X')) { x = GCode.GetWordValue('X'); }
              if (GCode.HasWord('Y')) { y = GCode.GetWordValue('Y'); }
              if (GCode.HasWord('Z')) { z = GCode.GetWordValue('Z'); }
              Serial.println("--------------");
              Serial.print(x);
              Serial.print("\t");
              Serial.print(y);
              Serial.print("\t");
              Serial.println(z);
              arm.goToCartesianPosition(x, y, z);
              break;
            }
          case 1:
            {
              double x = arm.x, y = arm.y, z = arm.z;
              Serial.print(x);
              Serial.print("\t");
              Serial.print(y);
              Serial.print("\t");
              Serial.println(z);
              if (GCode.HasWord('X')) { x = GCode.GetWordValue('X'); }
              if (GCode.HasWord('Y')) { y = GCode.GetWordValue('Y'); }
              if (GCode.HasWord('Z')) { z = GCode.GetWordValue('Z'); }
              Serial.println("--------------");
              Serial.print("Coordenadas cartesianas: ");
              Serial.print(x);
              Serial.print("\t");
              Serial.print(y);
              Serial.print("\t");
              Serial.println(z);
              arm.interpolateLine(x, y, z);
              break;
            }
          case 10:
            {
              arm.joint_2.goLimit(500);
              arm.goLimitSimultaneous();
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
              arm.goToAbsoluteArticularPosition(h, j, k, l);
              Serial.print("Coordenadas cartesianas: ");
              Serial.print(arm.x);
              Serial.print("\t");
              Serial.print(arm.y);
              Serial.print("\t");
              Serial.println(arm.z);
              Serial.print("Coordenadas articulares: ");
              Serial.print(h);
              Serial.print("\t");
              Serial.print(j);
              Serial.print("\t");
              Serial.print(k);
              Serial.print("\t");
              Serial.println(l);
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
              Serial.print("Coordenadas articulares:");
              Serial.print(h);
              Serial.print("\t");
              Serial.print(j);
              Serial.print("\t");
              Serial.print(k);
              Serial.print("\t");
              Serial.println(l);
              break;
            }
          case 20:
            {
              byte P = 255;
              if (GCode.HasWord('P')) { P = GCode.GetWordValue('P'); }
              arm.gripperServo.grab(P);
              break;
            }
          case 21:
            {
              arm.gripperServo.release();
              break;
            }
          default:
            break;
        }
      }
    }
  }
}