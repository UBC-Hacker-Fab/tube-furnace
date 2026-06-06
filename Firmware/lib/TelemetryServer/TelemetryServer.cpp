#include "TelemetryServer.h"

TelemetryServer::TelemetryServer(uint16_t port)
    : ws_(port), client_count_(0) {}

void TelemetryServer::begin() {
  ws_.begin();
  // Bind the C-style callback to this instance. arduinoWebSockets accepts a
  // std::function, so we can capture `this` and forward to a member.
  ws_.onEvent([this](uint8_t client, WStype_t type, uint8_t* payload,
                     size_t length) {
    handleEvent(client, type, payload, length);
  });
}

void TelemetryServer::loop() { ws_.loop(); }

void TelemetryServer::broadcast(const char* payload) {
  ws_.broadcastTXT(payload, strlen(payload));
}

void TelemetryServer::handleEvent(uint8_t client, WStype_t type,
                                  const uint8_t* /*payload*/,
                                  size_t /*length*/) {
  switch (type) {
    case WStype_CONNECTED:
      ++client_count_;
      Serial.printf("[WS] client #%u connected from %s (now %u)\n", client,
                    ws_.remoteIP(client).toString().c_str(), client_count_);
      break;
    case WStype_DISCONNECTED:
      if (client_count_ > 0) --client_count_;
      Serial.printf("[WS] client #%u disconnected (now %u)\n", client,
                    client_count_);
      break;
    default:
      break;  // Incoming frames are ignored: this is a one-way telemetry sink.
  }
}
