#pragma once

class PID {
public:
    PID(float kp, float ki, float kd, float minOut, float maxOut, float alphaD = 0.2f);
    float compute(float setpoint, float measurement, float dt);
    void reset();

private:
    float _kp, _ki, _kd;
    float _integral;
    float _previousError;
    float _alphaD;          // The smoothing factor (0.0 to 1.0)
    float _lastFilteredD;   // "dTermFilter" variable to store previous state
    float _minOut, _maxOut;
};