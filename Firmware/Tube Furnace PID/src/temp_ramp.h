#pragma once
#include <cstdint>

class TempRamp {
public:
    TempRamp(float finalTemp, uint32_t durationMs)
        : _final(finalTemp), _durationMs(durationMs) {}

    void begin(float startTemp, uint32_t nowMs) {
        _start   = startTemp;
        _startMs = nowMs;
        _active  = true;
    }

    float setpointAt(uint32_t nowMs) const {
        if (!_active) return _final;
        uint32_t elapsed = nowMs - _startMs;
        if (elapsed >= _durationMs) return _final;
        float frac = static_cast<float>(elapsed) / static_cast<float>(_durationMs);
        return _start + frac * (_final - _start);
    }

private:
    const float    _final;
    const uint32_t _durationMs;
    float    _start   = 0.0f;
    uint32_t _startMs = 0;
    bool     _active  = false;
};
