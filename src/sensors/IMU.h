#pragma once
#include <Wire.h>

class IMU
{
public:
    void begin();
    void calibrate();
    void loadCalibration();
    void update(float dt);
    void madgwick9DOF(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);

    float getRoll();
    float getPitch();
    float getYaw();

private:
    float gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
    float accAngleX_offset = 0, accAngleY_offset = 0;
    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // Quaternions
    const float beta_madgwick = 0.1f;                 // Algorithm gain
    float roll = 0;
    float pitch = 0;
    float yaw = 0;
};