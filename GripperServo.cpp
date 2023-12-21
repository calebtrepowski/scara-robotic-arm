#include "GripperServo.hpp"
#include "Constants.hpp"

GripperServo::GripperServo() {
}

GripperServo::~GripperServo() {}

void GripperServo::setup(const uint8_t pin, const uint8_t min, uint8_t max) {
  servo.attach(pin, min, max);
}

void GripperServo::grab(byte fraction_255 = 255) {
  DEBUG_PRINT("Closing gripper to %: ");
  DEBUG_PRINTLN(100 * fraction_255 / 255);
  servo.write(125 - 90 * fraction_255 / 255);
}

void GripperServo::release() {
  DEBUG_PRINTLN("Releasing gripper");
  servo.write(125);
}