#include <Arduino.h>
#include "furnace_task.h"
#include "wifi_task.h"

// Forward declarations (or add headers for the namespaces)
namespace FurnaceTask { void start(); }
namespace WiFiTask    { void start(); }

void setup() {
    Serial.begin(115200);
    delay(200);

    WiFiTask::start();     // Core 0
    FurnaceTask::start();  // Core 1
}

void loop() {
    vTaskDelete(nullptr);  // Arduino's loop() task is not needed
}
