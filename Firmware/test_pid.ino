#include <SPI.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

#define CS_PIN 5
#define SSR_PIN 13

#define Kp 0.2
#define Ki 0.0025
#define Kd 0.0

// --- WiFi credentials ---
#define WIFI_SSID     "your_ssid_here"
#define WIFI_PASSWORD "your_password_here"

WebServer server(80);

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

// --- HTTP handlers ---

void handleData() {
  char json[256];
  snprintf(json, sizeof(json),
    "{\"temperature\":%.2f,\"setpoint\":%.2f,\"duty_cycle\":%.4f,"
    "\"tc_connected\":%s,\"uptime_ms\":%lu}",
    latest.temperature, latest.setpoint, latest.dutyCycle,
    latest.tcConnected ? "true" : "false",
    latest.uptimeMs
  );
  server.send(200, "application/json", json);
}

void handleRoot() {
  server.send(200, "text/html",
    "<!DOCTYPE html><html><head>"
    "<meta charset='utf-8'><title>Tube Furnace</title>"
    "<style>"
    "body{font-family:monospace;background:#111;color:#eee;padding:2rem;}"
    "h1{color:#f90;} .card{background:#222;border-radius:8px;padding:1rem;margin:.5rem 0;}"
    ".label{color:#aaa;font-size:.85rem;} .val{font-size:2rem;font-weight:bold;}"
    ".ok{color:#4f4;} .warn{color:#f44;}"
    "</style></head><body>"
    "<h1>Tube Furnace Monitor</h1>"
    "<div class='card'><div class='label'>Temperature</div><div class='val' id='temp'>--</div></div>"
    "<div class='card'><div class='label'>Setpoint</div><div class='val' id='sp'>--</div></div>"
    "<div class='card'><div class='label'>Duty Cycle</div><div class='val' id='duty'>--</div></div>"
    "<div class='card'><div class='label'>Thermocouple</div><div class='val' id='tc'>--</div></div>"
    "<div class='card'><div class='label'>Uptime</div><div id='up'>--</div></div>"
    "<script>"
    "function fmt(s){const h=Math.floor(s/3600),m=Math.floor((s%3600)/60),sec=s%60;"
    "return `${h}h ${m}m ${sec}s`;}"
    "async function refresh(){"
    "try{const r=await fetch('/data');const d=await r.json();"
    "document.getElementById('temp').textContent=d.temperature.toFixed(1)+' °C';"
    "document.getElementById('sp').textContent=d.setpoint.toFixed(1)+' °C';"
    "document.getElementById('duty').textContent=(d.duty_cycle*100).toFixed(1)+'%';"
    "const tc=document.getElementById('tc');"
    "tc.textContent=d.tc_connected?'Connected':'DISCONNECTED';"
    "tc.className='val '+(d.tc_connected?'ok':'warn');"
    "document.getElementById('up').textContent=fmt(Math.floor(d.uptime_ms/1000));"
    "}catch(e){console.error(e);}}"
    "refresh();setInterval(refresh,1000);"
    "</script></body></html>"
  );
}

// --- END HTTP handlers ---

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

  // Start HTTP server
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

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
  server.handleClient();

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