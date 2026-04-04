#pragma once
#include <cstdint>

// Pins
constexpr uint8_t CS_PIN   = 5;
constexpr uint8_t SSR_PIN  = 13;
constexpr uint8_t SPI_CLK  = 18;
constexpr uint8_t SPI_MISO = 19;

// PID tuning
constexpr float Kp = 0.2f;
constexpr float Ki = 0.0025f;
constexpr float Kd = 0.0f;

// Temperature program
constexpr float    FINAL_SETPOINT   = 100.0f;
constexpr uint32_t RAMP_DURATION_MS = 30UL * 60UL * 1000UL;
constexpr uint32_t WINDOW_MS        = 1000;

// WiFi
constexpr char WIFI_SSID[]     = "your_ssid_here";
constexpr char WIFI_PASSWORD[] = "your_password_here";
constexpr uint16_t WS_PORT     = 81;

// FreeRTOS task config
constexpr uint32_t    FURNACE_STACK    = 4096;
constexpr uint32_t    WIFI_STACK       = 8192;
constexpr UBaseType_t FURNACE_PRIORITY = 3;   // Higher — time-critical
constexpr UBaseType_t WIFI_PRIORITY    = 1;
constexpr BaseType_t  FURNACE_CORE     = 1;   // Dedicated real-time core
constexpr BaseType_t  WIFI_CORE        = 0;   // WiFi stack lives on core 0
