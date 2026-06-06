// TimeProportionalActuator.h — Non-blocking time-proportional (slow-PWM)
// driver for an on/off actuator such as a solid-state relay (SSR).
//
// A heater driven through an SSR cannot be analog-dimmed; instead we switch it
// fully on for a fraction of a fixed window and off for the remainder. A duty
// cycle of 0.3 over a 1000 ms window means "on for 300 ms, off for 700 ms".
// This is the standard way to apply a continuous PID effort (0..1) to a binary
// actuator while switching slowly enough to be gentle on the relay.
//
// Crucially this implementation is *non-blocking*. The original firmware used
// delay(onTime)/delay(offTime), which froze the rest of the loop (WebSocket
// servicing, watchdog, etc.) for up to a full window. Here update() is called
// every loop iteration and simply compares millis() against the latched window
// boundaries — no delays.
//
// Usage:
//   TimeProportionalActuator heater(SSR_PIN, 1000);
//   heater.begin();
//   ...
//   heater.setDuty(pidOutput);   // 0.0 .. 1.0, applied at the next window
//   heater.update(millis());     // call every loop iteration

#pragma once

#include <Arduino.h>

class TimeProportionalActuator {
 public:
  // `pin`        GPIO driving the actuator.
  // `window_ms`  length of one on/off proportioning window.
  // `active_high` true if a HIGH level energizes the actuator (typical SSR).
  TimeProportionalActuator(uint8_t pin, unsigned long window_ms,
                           bool active_high = true);

  // Configure the pin and force the actuator off. Call once from setup().
  void begin();

  // Set the commanded duty cycle (clamped to 0..1). The new value is latched
  // at the start of the next window, so the current window completes cleanly.
  void setDuty(float duty);

  // Drive the output based on `now_ms` (pass millis()). Non-blocking; call as
  // often as possible. Handles window roll-over and latches the pending duty.
  void update(unsigned long now_ms);

  // Immediately de-energize the actuator and force duty to 0 (fail-safe).
  void forceOff();

  float duty() const { return active_duty_; }
  unsigned long windowMs() const { return window_ms_; }

 private:
  void writePin(bool energized);

  uint8_t pin_;
  unsigned long window_ms_;
  bool active_high_;

  float active_duty_;   // duty currently being applied this window
  float pending_duty_;  // duty to latch at the next window boundary
  unsigned long window_start_ms_;
  bool window_started_;
};
