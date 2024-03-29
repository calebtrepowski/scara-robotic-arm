#include <Arduino.h>

#define JOINT_1_STEP_PIN (2)
#define JOINT_1_DIRECTION_PIN (5)
#define JOINT_2_STEP_PIN (3)
#define JOINT_2_DIRECTION_PIN (6)
#define JOINT_3_STEP_PIN (4)
#define JOINT_3_DIRECTION_PIN (7)
#define JOINT_4_STEP_PIN (12)
#define JOINT_4_DIRECTION_PIN (13)
#define TOOL_SIGNAL_PIN (A0)
#define GRIPPER_SERVO_MIN_VALUE (600)
#define GRIPPER_SERVO_MAX_VALUE (2500)


#define JOINT_1_LIMIT_SWITCH_PIN (9)
#define JOINT_2_LIMIT_SWITCH_PIN (A3)
#define JOINT_3_LIMIT_SWITCH_PIN (11)
#define JOINT_4_LIMIT_SWITCH_PIN (10)

#define JOINT_1_LIMIT_DIRECTION (-1)
#define JOINT_2_LIMIT_DIRECTION (1)
#define JOINT_3_LIMIT_DIRECTION (-1)
#define JOINT_4_LIMIT_DIRECTION (-1)

#define JOINT_1_LIMIT_POSITION (-4000)
#define JOINT_2_LIMIT_POSITION (0)
#define JOINT_3_LIMIT_POSITION (-5420)
#define JOINT_4_LIMIT_POSITION (19)

#define JOINT_1_ANGLE_TO_STEPS (44.444444)
#define JOINT_2_DISTANCE_TO_STEPS (100)
#define JOINT_3_ANGLE_TO_STEPS (35.555555)
#define JOINT_4_ANGLE_TO_STEPS (10)

#define JOINT_1_MAX_ANGLE_DEGREES (345)
#define JOINT_2_MAX_DISTANCE_MM (240)
#define JOINT_3_MAX_ANGLE_DEGREES (345)
#define JOINT_4_MAX_ANGLE_DEGREES (345)

#define JOINT_1_MAX_STEPS_FROM_ZERO ((unsigned long)(JOINT_1_MAX_ANGLE_DEGREES * JOINT_1_ANGLE_TO_STEPS + JOINT_1_LIMIT_POSITION))
#define JOINT_2_MAX_STEPS_FROM_ZERO ((unsigned long)(JOINT_2_MAX_DISTANCE_MM * JOINT_2_DISTANCE_TO_STEPS))
#define JOINT_3_MAX_STEPS_FROM_ZERO ((unsigned long)(JOINT_3_MAX_ANGLE_DEGREES * JOINT_3_ANGLE_TO_STEPS + JOINT_3_LIMIT_POSITION))
#define JOINT_4_MAX_STEPS_FROM_ZERO ((unsigned long)(JOINT_4_MAX_ANGLE_DEGREES * JOINT_4_ANGLE_TO_STEPS))

#define L1 (double)(228)
// #define L1 (double)(220)
#define L2 (double)(136.5)
// #define L2 (double)(150)

#define DEBUG (1)
#define INFO (1)
#define ERROR (1)

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__);
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif

#if INFO
#define INFO_PRINT(...) Serial.print(__VA_ARGS__);
#define INFO_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
#define INFO_PRINT(...)
#define INFO_PRINTLN(...)
#endif

#if ERROR
#define ERROR_PRINT(...) Serial.print(__VA_ARGS__);
#define ERROR_PRINTLN(...) Serial.println(__VA_ARGS__);
#else
#define ERROR_PRINT(...)
#define ERROR_PRINTLN(...)
#endif