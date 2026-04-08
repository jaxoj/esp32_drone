#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <QMC5883LCompass.h>

class Compass {
public:
    Compass();
    
    // Initialize the sensor
    bool begin();
    
    // Read the hardware
    void update();
    
    // Calculate heading. 
    // Passing roll and pitch (in degrees) allows for tilt compensation.
    float getHeading(float rollDegrees, float pitchDegrees);

private:
    QMC5883LCompass _compass;
    int _x, _y, _z;

    // Hard-iron calibration offsets (you will tune these later)
    float _offsetX;
    float _offsetY;
    float _offsetZ;
};