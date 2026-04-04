#pragma once

class PIDController {
public:
    PIDController(float kp, float ki, float kd)
        : _kp(kp), _ki(ki), _kd(kd) {}

    void reset() {
        _integral    = 0.0f;
        _prevError   = 0.0f;
        _initialized = false;
    }

    float compute(float setpoint, float measured, float dtSec) {
        float error = setpoint - measured;

        if (!_initialized) {
            _prevError   = error;
            _initialized = true;
        }

        float P                = _kp * error;
        float candidateIntegral = _integral + _ki * error * dtSec;
        float D                = _kd * (error - _prevError) / dtSec;
        float uUnsat           = P + candidateIntegral + D;

        // Anti-windup: freeze integrator when saturated in the same direction as error
        bool windingUp   = (uUnsat > _outMax && error > 0.0f);
        bool windingDown = (uUnsat < _outMin && error < 0.0f);
        if (!windingUp && !windingDown) {
            _integral = clamp(candidateIntegral, _outMin, _outMax);
        }

        _prevError = error;
        return clamp(P + _integral + D, _outMin, _outMax);
    }

    void setOutputLimits(float min, float max) { _outMin = min; _outMax = max; }

private:
    static float clamp(float x, float lo, float hi) {
        return x < lo ? lo : (x > hi ? hi : x);
    }

    const float _kp, _ki, _kd;
    float _integral    = 0.0f;
    float _prevError   = 0.0f;
    float _outMin      = 0.0f;
    float _outMax      = 1.0f;
    bool  _initialized = false;
};
