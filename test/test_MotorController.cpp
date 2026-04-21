#include <unity.h>
#include "../src/actuators/MotorController.h"
#include "../src/config.h"

void test_motor_mixing_zero() {
    MotorController motors;
    // Don't call begin() to avoid hardware
    motors.update(1200, 0.0f, 0.0f, 0.0f);
    TEST_ASSERT_EQUAL_INT(1200, motors.getLastM1());
    TEST_ASSERT_EQUAL_INT(1200, motors.getLastM2());
    TEST_ASSERT_EQUAL_INT(1200, motors.getLastM3());
    TEST_ASSERT_EQUAL_INT(1200, motors.getLastM4());
}

void test_motor_mixing_roll() {
    MotorController motors;
    motors.update(1200, 100.0f, 0.0f, 0.0f); // roll +100
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM1()); // +roll
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM2()); // -roll
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM3()); // +roll
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM4()); // -roll
}

void test_motor_mixing_pitch() {
    MotorController motors;
    motors.update(1200, 0.0f, 100.0f, 0.0f); // pitch +100
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM1()); // -pitch
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM2()); // -pitch
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM3()); // +pitch
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM4()); // +pitch
}

void test_motor_mixing_yaw() {
    MotorController motors;
    motors.update(1200, 0.0f, 0.0f, 100.0f); // yaw +100
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM1()); // +yaw
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM2()); // -yaw
    TEST_ASSERT_EQUAL_INT(1100, motors.getLastM3()); // -yaw
    TEST_ASSERT_EQUAL_INT(1300, motors.getLastM4()); // +yaw
}

void test_motor_constrain() {
    MotorController motors;
    motors.update(1200, 1000.0f, 1000.0f, 1000.0f); // large values
    TEST_ASSERT_EQUAL_INT(PWM_MAX, motors.getLastM1());
    TEST_ASSERT_EQUAL_INT(PWM_MIN, motors.getLastM2()); // 1200 -1000 -1000 -1000 = -1600 -> 1000
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_motor_mixing_zero);
    RUN_TEST(test_motor_mixing_roll);
    RUN_TEST(test_motor_mixing_pitch);
    RUN_TEST(test_motor_mixing_yaw);
    RUN_TEST(test_motor_constrain);
    UNITY_END();
}

void loop() {
    // Nothing
}