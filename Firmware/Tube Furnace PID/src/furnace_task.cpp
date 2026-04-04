#include "config.h"
#include "shared_data.h"
#include "thermocouple.h"
#include "pid_controller.h"
#include "temp_ramp.h"
#include <SPI.h>
#include <Arduino.h>

namespace FurnaceTask {

static SPIClass      spi(VSPI);
static MAX6675       tc(spi, CS_PIN);
static PIDController pid(Kp, Ki, Kd);
static TempRamp      ramp(FINAL_SETPOINT, RAMP_DURATION_MS);

static void taskFn(void*) {
    spi.begin(SPI_CLK, SPI_MISO, -1, CS_PIN);
    tc.begin();
    pid.reset();

    pinMode(SSR_PIN, OUTPUT);
    digitalWrite(SSR_PIN, LOW);

    // Block until thermocouple gives a valid reading
    float startTemp = NAN;
    while (isnan(startTemp)) {
        startTemp = tc.read();
        vTaskDelay(pdMS_TO_TICKS(250));
    }
    ramp.begin(startTemp, millis());
    Serial.println("[Furnace] Running. Temp,Setpoint,Duty");

    while (true) {
        TickType_t windowStart = xTaskGetTickCount();
        uint32_t   nowMs       = pdTICKS_TO_MS(windowStart);

        float temp = tc.read();
        SensorSnapshot snap{ .uptimeMs = nowMs };

        if (isnan(temp)) {
            snap.tcConnected = false;
            digitalWrite(SSR_PIN, LOW);
            SharedData::instance().write(snap);
            vTaskDelayUntil(&windowStart, pdMS_TO_TICKS(WINDOW_MS));
            continue;
        }

        float setpoint  = ramp.setpointAt(nowMs);
        float dutyCycle = pid.compute(setpoint, temp, WINDOW_MS / 1000.0f);

        snap = { temp, setpoint, dutyCycle, true, nowMs };
        SharedData::instance().write(snap);
        Serial.printf("%.2f,%.2f,%.4f\n", temp, setpoint, dutyCycle);

        // Time-proportioning: SSR on for dutyCycle fraction of the window
        uint32_t onMs = static_cast<uint32_t>(WINDOW_MS * dutyCycle);
        if (onMs > 0) {
            digitalWrite(SSR_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(onMs));
        }
        digitalWrite(SSR_PIN, LOW);

        // Wait out the rest of the window (absorbs computation time too)
        vTaskDelayUntil(&windowStart, pdMS_TO_TICKS(WINDOW_MS));
    }
}

void start() {
    xTaskCreatePinnedToCore(taskFn, "furnace", FURNACE_STACK,
                            nullptr, FURNACE_PRIORITY, nullptr, FURNACE_CORE);
}

} // namespace FurnaceTask
