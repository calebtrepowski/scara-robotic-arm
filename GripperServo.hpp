#include <Arduino.h>
#include <Servo.h>
class GripperServo {
public:
  GripperServo();
  ~GripperServo();

  void setup(const uint8_t pin, const uint8_t min, uint8_t max);

  void grab(byte fraction_255 = 255);
  void release(void);

  friend void setup(void);

private:
  Servo servo;
};