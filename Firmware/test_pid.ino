#include <SPI.h>
#include <math.h>
#include <WiFi.h>
#include <WebSocketsServer.h>

#define CS_PIN 5
#define SSR_PIN 13

#define Kp 0.2
#define Ki 0.0025
#define Kd 0.0

// --- WiFi credentials ---
#define WIFI_SSID     "your_ssid_here"
#define WIFI_PASSWORD "your_password_here"

WebSocketsServer webSocket(81);

SPIClass spi(VSPI);

// SSR time-proportioning settings
const unsigned long windowMs = 1000;   // 2 second control window

// Temperature program
const float finalSetpoint = 100.0;                 // deg C
const unsigned long rampDurationMs = 30UL * 60UL * 1000UL;  // 15 minutes

// PID state
float integralTerm = 0.0;
float prevError = 0.0;
bool pidInitialized = false;

// Ramp state
float startTemp = 25.0;
unsigned long rampStartMs = 0;
bool rampInitialized = false;

// Latest sensor data for WiFi broadcast
struct SensorData {
  float temperature;
  float setpoint;
  float dutyCycle;
  bool tcConnected;
  unsigned long uptimeMs;
} latest = {0.0, 0.0, 0.0, false, 0};

// --- WebSocket handler ---

void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] Client #%u connected from %s\n", num,
                    webSocket.remoteIP(num).toString().c_str());
      break;
    case WStype_DISCONNECTED:
      Serial.printf("[WS] Client #%u disconnected\n", num);
      break;
    default: break;
  }
}

// --- END WebSocket handler ---

float readMAX6675() {
  uint16_t data;

  digitalWrite(CS_PIN, LOW);
  delayMicroseconds(5);
  data = spi.transfer16(0x0000);
  digitalWrite(CS_PIN, HIGH);

  if (data & 0x4) {
    // Serial.println("Thermocouple disconnected");
    return NAN;
  }

  data >>= 3;
  return data * 0.25;
}

float clampf(float x, float xmin, float xmax) {
  if (x < xmin) return xmin;
  if (x > xmax) return xmax;
  return x;
}

float computeRampSetpoint(unsigned long nowMs) {
  if (!rampInitialized) {
    return finalSetpoint;
  }

  unsigned long elapsedMs = nowMs - rampStartMs;

  if (elapsedMs >= rampDurationMs) {
    return finalSetpoint;
  }

  float frac = (float)elapsedMs / (float)rampDurationMs;
  return startTemp + frac * (finalSetpoint - startTemp);
}

float computePID(float setpoint, float measured, float dtSec) {
  float error = setpoint - measured;

  if (!pidInitialized) {
    prevError = error;
    pidInitialized = true;
  }

  // Proportional
  float P = Kp * error;

  // Integral candidate
  float newIntegralTerm = integralTerm + Ki * error * dtSec;

  // Derivative
  float derivative = (error - prevError) / dtSec;
  float D = Kd * derivative;

  // Unsaturated output using candidate integral
  float uUnsat = P + newIntegralTerm + D;

  // Saturate to duty-cycle limits
  float uSat = clampf(uUnsat, 0.0, 1.0);

  // Simple anti-windup:
  // only accept the new integral if we're not saturating further in the wrong direction
  bool allowIntegrate = true;

  if (uUnsat > 1.0 && error > 0.0) allowIntegrate = false;
  if (uUnsat < 0.0 && error < 0.0) allowIntegrate = false;

  if (allowIntegrate) {
    integralTerm = newIntegralTerm;
  }

  // Optional extra clamp on integral term itself for safety
  integralTerm = clampf(integralTerm, -1.0, 1.0);

  // Recompute with accepted integral
  float output = P + integralTerm + D;
  output = clampf(output, 0.0, 1.0);

  prevError = error;
  return output;
}

void setup() {
  Serial.begin(115200);

  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  spi.begin(18, 19, -1, CS_PIN);

  pinMode(SSR_PIN, OUTPUT);
  digitalWrite(SSR_PIN, LOW);

  delay(500);

  // Connect to WiFi
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("\nWiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi failed — running without network.");
  }

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);

  // Initialize ramp from first valid measurement
  float temp = readMAX6675();
  while (isnan(temp)) {
    delay(500);
    temp = readMAX6675();
  }

  startTemp = temp;
  rampStartMs = millis();
  rampInitialized = true;

  Serial.println("Temp,Setpoint,Duty");
}

void loop() {
  webSocket.loop();

  unsigned long windowStart = millis();

  float temp = readMAX6675();
  if (isnan(temp)) {
    latest.tcConnected = false;
    latest.uptimeMs = windowStart;
    digitalWrite(SSR_PIN, LOW);
    delay(windowMs);
    return;
  }

  float setpoint = computeRampSetpoint(windowStart);
  float dtSec = windowMs / 1000.0;

  float dutyCycle = computePID(setpoint, temp, dtSec);
  dutyCycle = clampf(dutyCycle, 0.0, 1.0);

  // Update latest data for WiFi clients
  latest.temperature = temp;
  latest.setpoint    = setpoint;
  latest.dutyCycle   = dutyCycle;
  latest.tcConnected = true;
  latest.uptimeMs    = windowStart;

  // Broadcast sensor JSON to all WebSocket clients
  char json[256];
  snprintf(json, sizeof(json),
    "{\"temperature\":%.2f,\"setpoint\":%.2f,\"duty_cycle\":%.4f,"
    "\"tc_connected\":%s,\"uptime_ms\":%lu}",
    latest.temperature, latest.setpoint, latest.dutyCycle,
    latest.tcConnected ? "true" : "false",
    latest.uptimeMs
  );
  webSocket.broadcastTXT(json, strlen(json));

  unsigned long onTime  = (unsigned long)(windowMs * dutyCycle);
  unsigned long offTime = windowMs - onTime;

  Serial.print(temp); Serial.print(",");
  Serial.print(setpoint); Serial.print(",");
  Serial.println(dutyCycle, 4);

  if (onTime > 0) {
    digitalWrite(SSR_PIN, HIGH);
    delay(onTime);
  }

  if (offTime > 0) {
    digitalWrite(SSR_PIN, LOW);
    delay(offTime);
  } else {
    digitalWrite(SSR_PIN, LOW);
  }
}