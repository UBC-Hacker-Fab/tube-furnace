// main.cpp — Tube furnace controller application.
//
// Responsibilities kept here (everything reusable lives in lib/):
//   * Bring up hardware: SPI bus, thermocouple, SSR, WiFi, WebSocket server.
//   * Generate the setpoint program (linear ramp, then hold).
//   * Run the control loop once per window: read temperature -> PID -> duty.
//   * Stream telemetry as JSON over WebSockets and CSV over Serial.
//
// The loop is fully non-blocking: the SSR is time-proportioned by
// TimeProportionalActuator and the WebSocket server is serviced every
// iteration, so neither starves the other.

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <math.h>

#include "config.h"
#include "secrets.h"

#include "MAX6675.h"
#include "PIDController.h"
#include "TelemetryServer.h"
#include "TimeProportionalActuator.h"

// ---------------------------------------------------------------------------
// Peripherals
// ---------------------------------------------------------------------------
SPIClass spi(VSPI);
MAX6675 thermocouple(spi, pins::kThermocoupleCs);
TimeProportionalActuator heater(pins::kSolidStateRelay, kControlWindowMs);
TelemetryServer telemetry(net::kWebSocketPort);
PIDController controller({pid::kKp, pid::kKi, pid::kKd}, /*output_min=*/0.0f,
                         /*output_max=*/1.0f);

// ---------------------------------------------------------------------------
// Setpoint program: linear ramp from the boot temperature to the final
// setpoint over kRampDurationMs, then hold.
// ---------------------------------------------------------------------------
struct LinearRamp {
  float start_c = 25.0f;
  unsigned long start_ms = 0;

  void begin(float start_temp_c, unsigned long now_ms) {
    start_c = start_temp_c;
    start_ms = now_ms;
  }

  float valueAt(unsigned long now_ms) const {
    const unsigned long elapsed = now_ms - start_ms;
    if (elapsed >= program::kRampDurationMs) return program::kFinalSetpointC;
    const float frac =
        static_cast<float>(elapsed) / static_cast<float>(program::kRampDurationMs);
    return start_c + frac * (program::kFinalSetpointC - start_c);
  }
} ramp;

// Timestamp of the last control update; drives the once-per-window cadence and
// the dt handed to the PID.
unsigned long last_control_ms = 0;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void connectWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < net::kWifiConnectAttempts &&
                  WiFi.status() != WL_CONNECTED;
       ++i) {
    delay(net::kWifiAttemptDelayMs);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed -- running without network.");
  }
}

static void broadcastTelemetry(float temperature, float setpoint, float duty,
                               bool tc_connected, unsigned long uptime_ms) {
  char json[192];
  snprintf(json, sizeof(json),
           "{\"temperature\":%.2f,\"setpoint\":%.2f,\"duty_cycle\":%.4f,"
           "\"tc_connected\":%s,\"uptime_ms\":%lu}",
           temperature, setpoint, duty, tc_connected ? "true" : "false",
           uptime_ms);
  telemetry.broadcast(json);
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // SPI bus for the MAX6675 (read-only: no MOSI).
  spi.begin(pins::kSpiSck, pins::kSpiMiso, /*MOSI=*/-1, pins::kThermocoupleCs);
  thermocouple.begin();

  heater.begin();  // SSR forced off

  delay(500);

  connectWiFi();
  telemetry.begin();

  // Initialize the ramp from the first valid temperature reading.
  float temp = thermocouple.readCelsius();
  while (isnan(temp)) {
    Serial.println("Waiting for thermocouple...");
    delay(500);
    temp = thermocouple.readCelsius();
  }
  ramp.begin(temp, millis());
  last_control_ms = millis();

  Serial.println("Temp,Setpoint,Duty");
}

void loop() {
  const unsigned long now = millis();

  // Service the network and drive the SSR every iteration -- both non-blocking.
  telemetry.loop();
  heater.update(now);

  // Run the control law once per control window.
  if (now - last_control_ms < kControlWindowMs) return;

  const float dt_sec = (now - last_control_ms) / 1000.0f;
  last_control_ms = now;

  const float temp = thermocouple.readCelsius();
  if (isnan(temp)) {
    // Fail safe: open thermocouple -> cut the heater and report the fault.
    heater.forceOff();
    broadcastTelemetry(NAN, ramp.valueAt(now), 0.0f, /*tc_connected=*/false,
                       now);
    Serial.println("nan,nan,0.0000");
    return;
  }

  const float setpoint = ramp.valueAt(now);
  const float duty = controller.Update(setpoint, temp, dt_sec);
  heater.setDuty(duty);

  broadcastTelemetry(temp, setpoint, duty, /*tc_connected=*/true, now);

  Serial.print(temp);
  Serial.print(",");
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(duty, 4);
}
