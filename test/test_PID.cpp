#include <unity.h>
#include "../src/control/PID.h"

void test_pid_initialization() {
    PID pid(1.0f, 0.1f, 0.01f, -10.0f, 10.0f);
    // Test that PID is initialized
    TEST_ASSERT_EQUAL_FLOAT(1.0f, pid.compute(0.0f, 0.0f, 0.1f)); // P term only, error 0
}

void test_pid_proportional() {
    PID pid(2.0f, 0.0f, 0.0f, -100.0f, 100.0f);
    float output = pid.compute(5.0f, 0.0f, 0.1f); // setpoint 5, measurement 0, error 5
    TEST_ASSERT_EQUAL_FLOAT(10.0f, output); // 2 * 5
}

void test_pid_integral() {
    PID pid(0.0f, 1.0f, 0.0f, -100.0f, 100.0f);
    pid.compute(1.0f, 0.0f, 0.1f); // error 1, dt 0.1, integral += 0.1
    float output = pid.compute(1.0f, 0.0f, 0.1f); // integral += 0.1, total 0.2
    TEST_ASSERT_EQUAL_FLOAT(0.2f, output);
}

void test_pid_derivative() {
    PID pid(0.0f, 0.0f, 1.0f, -100.0f, 100.0f);
    pid.compute(0.0f, 0.0f, 0.1f); // prev error 0
    float output = pid.compute(1.0f, 0.0f, 0.1f); // error 1, prev 0, derivative (1-0)/0.1 = 10
    TEST_ASSERT_EQUAL_FLOAT(10.0f, output);
}

void test_pid_reset() {
    PID pid(1.0f, 1.0f, 1.0f, -100.0f, 100.0f);
    pid.compute(1.0f, 0.0f, 0.1f);
    pid.reset();
    float output = pid.compute(1.0f, 0.0f, 0.1f); // should be fresh
    TEST_ASSERT_EQUAL_FLOAT(1.0f, output); // P only
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_pid_initialization);
    RUN_TEST(test_pid_proportional);
    RUN_TEST(test_pid_integral);
    RUN_TEST(test_pid_derivative);
    RUN_TEST(test_pid_reset);
    UNITY_END();
}

void loop() {
    // Nothing
}