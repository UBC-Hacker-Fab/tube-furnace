#include "config.h"
#include "shared_data.h"
#include <WiFi.h>
#include <WebSocketsServer.h>
#include <Arduino.h>

namespace WiFiTask {

static WebSocketsServer ws(WS_PORT);

static void onWsEvent(uint8_t num, WStype_t type, uint8_t*, size_t) {
    if (type == WStype_CONNECTED)
        Serial.printf("[WS] Client #%u connected\n", num);
    else if (type == WStype_DISCONNECTED)
        Serial.printf("[WS] Client #%u disconnected\n", num);
}

static void taskFn(void*) {
    Serial.print("[WiFi] Connecting");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
        vTaskDelay(pdMS_TO_TICKS(500));
        Serial.print(".");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Failed — shutting down WiFi task.");
        vTaskDelete(nullptr);
        return;
    }
    Serial.printf("\n[WiFi] Connected. IP: %s\n", WiFi.localIP().toString().c_str());

    ws.begin();
    ws.onEvent(onWsEvent);

    char json[256];
    while (true) {
        ws.loop();

        SensorSnapshot snap = SharedData::instance().read();
        snprintf(json, sizeof(json),
            "{\"temperature\":%.2f,\"setpoint\":%.2f,\"duty_cycle\":%.4f,"
            "\"tc_connected\":%s,\"uptime_ms\":%lu}",
            snap.temperature, snap.setpoint, snap.dutyCycle,
            snap.tcConnected ? "true" : "false",
            static_cast<unsigned long>(snap.uptimeMs));

        ws.broadcastTXT(json, strlen(json));
        vTaskDelay(pdMS_TO_TICKS(200));  // ~5 Hz broadcast rate
    }
}

void start() {
    xTaskCreatePinnedToCore(taskFn, "wifi_ws", WIFI_STACK,
                            nullptr, WIFI_PRIORITY, nullptr, WIFI_CORE);
}

} // namespace WiFiTask
