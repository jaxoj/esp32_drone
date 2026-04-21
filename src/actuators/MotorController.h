#pragma once
#include <ESP32Servo.h>

class MotorController {
public:
    void begin();
    void arm();

    void update(int baseThrottle, float roll, float pitch, float yaw);

    // For testing
    int getLastM1() { return lastM1; }
    int getLastM2() { return lastM2; }
    int getLastM3() { return lastM3; }
    int getLastM4() { return lastM4; }

private:
    Servo esc1, esc2, esc3, esc4;
    int lastM1, lastM2, lastM3, lastM4;
};