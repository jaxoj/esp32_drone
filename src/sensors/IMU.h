#pragma once
#include <Wire.h>

class IMU
{
public:
    void begin();
    void calibrate();

    void update(float dt);

    float getRoll();
    float getPitch();
    float getYaw();
    

private:
    float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
    float accAngleX_offset = 0, accAngleY_offset = 0;
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};