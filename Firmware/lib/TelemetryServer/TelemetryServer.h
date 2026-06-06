// TelemetryServer.h — Thin wrapper around arduinoWebSockets that turns a
// WebSocket server into a one-line "broadcast telemetry" sink.
//
// This library owns the socket lifecycle (accept, connect/disconnect logging,
// servicing) so the application loop only has to:
//
//   TelemetryServer telemetry(81);
//   telemetry.begin();
//   ...
//   telemetry.loop();              // call every iteration, non-blocking
//   telemetry.broadcast(jsonStr);  // push a frame to all connected clients
//
// It is payload-agnostic: callers format whatever text they like (JSON, CSV,
// ...) and hand it over. Nothing here is furnace-specific, so it can be reused
// by any project that needs to stream data to browser/desktop clients.
//
// Depends on links2004/WebSockets.

#pragma once

#include <Arduino.h>
#include <WebSocketsServer.h>

class TelemetryServer {
 public:
  explicit TelemetryServer(uint16_t port = 81);

  // Start listening. Call once from setup() after the network is up.
  void begin();

  // Service the server: must be called frequently from loop(). Non-blocking.
  void loop();

  // Send a NUL-terminated text frame to every connected client. Safe to call
  // with no clients connected (it's a no-op).
  void broadcast(const char* payload);

  // Number of currently connected clients.
  uint8_t clientCount() const { return client_count_; }

 private:
  void handleEvent(uint8_t client, WStype_t type, const uint8_t* payload,
                   size_t length);

  WebSocketsServer ws_;
  uint8_t client_count_;
};
