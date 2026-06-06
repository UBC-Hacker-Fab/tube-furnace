// config.h — Hardware pin map and process parameters for the tube furnace.
//
// This is the single place to tune the controller for a given build of the
// furnace. WiFi credentials live separately in secrets.h (gitignored) so this
// file can be committed safely.

#pragma once

// ---------------------------------------------------------------------------
// Pin map (ESP-WROOM-32, VSPI bus)
// ---------------------------------------------------------------------------
// The MAX6675 is a read-only SPI device, so no MOSI line is wired.
namespace pins {
constexpr int kThermocoupleCs = 5;    // MAX6675 chip select
constexpr int kSpiSck         = 18;   // MAX6675 SCK
constexpr int kSpiMiso        = 19;   // MAX6675 SO  (data out of the sensor)
constexpr int kSolidStateRelay = 13;  // SSR gate driving the heater
}  // namespace pins

// ---------------------------------------------------------------------------
// PID gains (output is heater duty cycle, 0.0 .. 1.0)
// ---------------------------------------------------------------------------
namespace pid {
constexpr float kKp = 0.2f;
constexpr float kKi = 0.0025f;
constexpr float kKd = 0.0f;
}  // namespace pid

// ---------------------------------------------------------------------------
// Control timing
// ---------------------------------------------------------------------------
// One control window: the PID is evaluated once per window and the SSR is
// time-proportioned across it. Keep this comfortably longer than the MAX6675
// conversion time (~0.22 s) and a multiple of the AC line period for clean SSR
// switching.
constexpr unsigned long kControlWindowMs = 1000;

// ---------------------------------------------------------------------------
// Temperature program: linear ramp from the starting temperature (sampled at
// boot) up to the final setpoint, then hold.
// ---------------------------------------------------------------------------
namespace program {
constexpr float kFinalSetpointC = 100.0f;
constexpr unsigned long kRampDurationMs = 30UL * 60UL * 1000UL;  // 30 minutes
}  // namespace program

// ---------------------------------------------------------------------------
// Network
// ---------------------------------------------------------------------------
namespace net {
constexpr uint16_t kWebSocketPort = 81;
constexpr int kWifiConnectAttempts = 20;       // x 500 ms = 10 s max wait
constexpr unsigned long kWifiAttemptDelayMs = 500;
}  // namespace net
