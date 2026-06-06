#include "TimeProportionalActuator.h"

namespace {
float Clamp01(float x) {
  if (x < 0.0f) return 0.0f;
  if (x > 1.0f) return 1.0f;
  return x;
}
}  // namespace

TimeProportionalActuator::TimeProportionalActuator(uint8_t pin,
                                                   unsigned long window_ms,
                                                   bool active_high)
    : pin_(pin),
      window_ms_(window_ms),
      active_high_(active_high),
      active_duty_(0.0f),
      pending_duty_(0.0f),
      window_start_ms_(0),
      window_started_(false) {}

void TimeProportionalActuator::begin() {
  pinMode(pin_, OUTPUT);
  writePin(false);
}

void TimeProportionalActuator::setDuty(float duty) {
  pending_duty_ = Clamp01(duty);
}

void TimeProportionalActuator::forceOff() {
  active_duty_ = 0.0f;
  pending_duty_ = 0.0f;
  writePin(false);
}

void TimeProportionalActuator::update(unsigned long now_ms) {
  // Start the first window, or roll over into a new one once the current
  // window has fully elapsed. (unsigned subtraction is millis()-rollover safe.)
  if (!window_started_ || (now_ms - window_start_ms_) >= window_ms_) {
    window_start_ms_ = now_ms;
    window_started_ = true;
    active_duty_ = pending_duty_;  // latch the most recent command
  }

  const unsigned long elapsed = now_ms - window_start_ms_;
  const unsigned long on_time = (unsigned long)(window_ms_ * active_duty_);

  // Energized for the leading `on_time` of the window, off for the rest.
  writePin(elapsed < on_time);
}

void TimeProportionalActuator::writePin(bool energized) {
  digitalWrite(pin_, (energized == active_high_) ? HIGH : LOW);
}
