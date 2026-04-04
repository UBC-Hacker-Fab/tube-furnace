#pragma once
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <cstdint>

struct SensorSnapshot {
    float    temperature = 0.0f;
    float    setpoint    = 0.0f;
    float    dutyCycle   = 0.0f;
    bool     tcConnected = false;
    uint32_t uptimeMs    = 0;
};

// Singleton: PID task writes, WiFi task reads, mutex-protected
class SharedData {
public:
    static SharedData& instance() {
        static SharedData inst;
        return inst;
    }

    void write(const SensorSnapshot& snap) {
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            _snap = snap;
            xSemaphoreGive(_mutex);
        }
    }

    SensorSnapshot read() {
        SensorSnapshot out;
        if (xSemaphoreTake(_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            out = _snap;
            xSemaphoreGive(_mutex);
        }
        return out;
    }

private:
    SharedData() { _mutex = xSemaphoreCreateMutex(); }
    SemaphoreHandle_t _mutex;
    SensorSnapshot    _snap;
};
