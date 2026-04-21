#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd, float minOut, float maxOut, float alphaD)
    : _kp(kp), _ki(ki), _kd(kd),
      _integral(0), _previousError(0),
      _alphaD(alphaD), _lastFilteredD(0),
      _minOut(minOut), _maxOut(maxOut) {}

float PID::compute(float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    // 1. Proportional
    float p_out = _kp * error;

    // 2. Integral (with Windup protection)
    _integral += error * dt;
    _integral = constrain(_integral, _minOut, _maxOut);
    float i_out = _ki * _integral;

    // 3. Derivative with Low-Pass Filter (Issue #2)
    float d_out = 0;
    if (dt > 0.0001f) {
        // Calculate Raw Derivative
        float raw_derivative = (error - _previousError) / dt;

        /* Low Pass Filter Formula: 
           Filtered = (Alpha * New) + ((1 - Alpha) * Old)
           A lower Alpha (e.g. 0.1) = More Filtering (Smoother but slower)
           A higher Alpha (e.g. 0.5) = Less Filtering (Faster but noisier)
        */
        float current_filtered_d = (_alphaD * raw_derivative) + ((1.0f - _alphaD) * _lastFilteredD);
        
        d_out = _kd * current_filtered_d;

        // Store current filtered value for the next iteration
        _lastFilteredD = current_filtered_d;
    }

    // 4. Final Output
    float output = p_out + i_out + d_out;
    output = constrain(output, _minOut, _maxOut);

    _previousError = error;

    return output;
}

void PID::reset()
{
    _integral = 0;
    _previousError = 0;
    _lastFilteredD = 0;
}