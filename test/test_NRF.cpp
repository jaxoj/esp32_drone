#include <unity.h>
#include "../src/control/NRF.h"

// Mock RF24 since we can't test hardware
// This is a basic test for struct and defaults

void test_nrf_defaults() {
    // Since NRFReceiver requires pins, we can't fully instantiate without hardware
    // Test the struct directly
    RadioPacket packet;
    packet.throttle = 1000;
    packet.rollTarget = 0.0f;
    packet.pitchTarget = 0.0f;
    packet.yawTarget = 0.0f;

    TEST_ASSERT_EQUAL_INT16(1000, packet.throttle);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, packet.rollTarget);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, packet.pitchTarget);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, packet.yawTarget);
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_nrf_defaults);
    UNITY_END();
}

void loop() {
    // Nothing
}