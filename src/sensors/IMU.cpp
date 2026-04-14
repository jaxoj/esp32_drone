#include "IMU.h"
#include "../config.h"
#include <Arduino.h>

void IMU::begin() {
    // 1. Wake up MPU
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);

    // 2. Enable Hardware Digital Low Pass Filter (DLPF)
    // Register 0x1A: Configures the LPF. 
    // Value 0x03 (~42Hz) is the "sweet spot" for drone vibrations.
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1A); 
    Wire.write(0x04); 
    Wire.endTransmission(true);

    // 3. Set Gyro Full Scale Range to +/- 500 deg/s (Optional but recommended)
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x1B);
    Wire.write(0x08); 
    Wire.endTransmission(true);
}

void IMU::calibrate() {
    Serial.println("Calibrating IMU...");
    gyroX_offset = 0; gyroY_offset = 0; gyroZ_offset = 0;
    
    for (int i = 0; i < 1000; i++) { // Increased to 1000 for better precision
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU_ADDR, 6, true);

        gyroX_offset += (int16_t(Wire.read() << 8 | Wire.read())) / 65.5; // Scale for 500dps
        gyroY_offset += (int16_t(Wire.read() << 8 | Wire.read())) / 65.5;
        gyroZ_offset += (int16_t(Wire.read() << 8 | Wire.read())) / 65.5;
        delay(2);
    }

    gyroX_offset /= 1000.0;
    gyroY_offset /= 1000.0;
    gyroZ_offset /= 1000.0;
    Serial.println("IMU Calibrated.");
}

void IMU::update(float dt) {
    // --- Read Raw Data ---
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);

    float ax = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
    float ay = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
    float az = (int16_t(Wire.read() << 8 | Wire.read())) / 16384.0;
    Wire.read(); Wire.read(); // Skip Temp
    // Note: If you didn't change range to 500dps in begin(), keep using 131.0
    float gx = ((int16_t(Wire.read() << 8 | Wire.read())) / 65.5) - gyroX_offset;
    float gy = ((int16_t(Wire.read() << 8 | Wire.read())) / 65.5) - gyroY_offset;
    float gz = ((int16_t(Wire.read() << 8 | Wire.read())) / 65.5) - gyroZ_offset;

    // --- Madgwick Algorithm (Optimized for IMU) ---
    // Convert gyro to radians per second
    float gx_rad = gx * PI / 180.0f;
    float gy_rad = gy * PI / 180.0f;
    float gz_rad = gz * PI / 180.0f;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx_rad - q2 * gy_rad - q3 * gz_rad);
    qDot2 = 0.5f * (q0 * gx_rad + q2 * gz_rad - q3 * gy_rad);
    qDot3 = 0.5f * (q0 * gy_rad - q1 * gz_rad + q3 * gx_rad);
    qDot4 = 0.5f * (q0 * gz_rad + q1 * gy_rad - q2 * gx_rad);

    // Compute feedback only if accelerometer measurement valid (avoids NaN)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0; _2q1 = 2.0f * q1; _2q2 = 2.0f * q2; _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0; _4q1 = 4.0f * q1; _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1; _8q2 = 8.0f * q2;
        q0q0 = q0 * q0; q1q1 = q1 * q1; q2q2 = q2 * q2; q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        
        recipNorm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

        // Apply feedback
        qDot1 -= beta_madgwick * s0;
        qDot2 -= beta_madgwick * s1;
        qDot3 -= beta_madgwick * s2;
        qDot4 -= beta_madgwick * s3;
    }

    // Integrate rate of change to yield quaternion
    q0 += qDot1 * dt; q1 += qDot2 * dt; q2 += qDot3 * dt; q3 += qDot4 * dt;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;

    // --- Convert Quaternion to Euler Angles ---
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 180.0f / PI;
    pitch = asin(2.0f * (q0 * q2 - q1 * q3)) * 180.0f / PI;
    yaw = gz; // Keeping Yaw as Rate Control as discussed

    Serial.printf("Roll: %3.1f | Pitch: %3.1f | Yaw: %3.1f\n", roll, pitch, yaw);
}

float IMU::getRoll() { return roll; }
float IMU::getPitch() { return pitch; }
float IMU::getYaw() { return yaw; }