// src/sensors/Compass.cpp
#include "Compass.h"
#include <math.h>

Compass::Compass() {}

bool Compass::begin()
{
    _compass.init();
    return true;
}

void Compass::update()
{
    _compass.read();

    _x = _compass.getX();
    _y = _compass.getY();
    _z = _compass.getZ();
}

float Compass::getHeading(float rollDegrees, float pitchDegrees)
{
    // Apply Calibration Offsets
    float x = _x - _offsetX;
    float y = _y - _offsetY;
    float z = _z - _offsetZ;

    // Convert degrees to radians for the math functions
    float roll = rollDegrees * (PI / 180.0);
    float pitch = pitchDegrees * (PI / 180.0);

    // Tilt Compensation Math
    // If the drone is tilted, the magnetic X and Y vectors change.
    // This math projects the tilted vectors back onto a flat horizontal plane.
    float cosRoll = cos(roll);
    float sinRoll = sin(roll);
    float cosPitch = cos(pitch);
    float sinPitch = sin(pitch);

    float Xh = x * cosPitch + z * sinPitch;
    float Yh = x * sinRoll * sinPitch + y * cosRoll - z * sinRoll * cosPitch;

    // Calculate Heading
    float heading = atan2(Yh, Xh);

    // Correct for negative values (atan2 returns -PI to +PI)
    if (heading < 0)
    {
        heading += 2 * PI;
    }

    // Convert radians back to degrees
    float headingDegrees = heading * (180.0 / PI);

    return headingDegrees;
}