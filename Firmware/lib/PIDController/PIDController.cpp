#include "PIDController.h"

namespace {
float Clamp(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}
}  // namespace

PIDController::PIDController(const Gains& gains, float output_min,
                            float output_max)
    : gains_(gains),
      output_min_(output_min),
      output_max_(output_max),
      integrator_(0.0f),
      prev_error_(0.0f),
      initialized_(false) {}

void PIDController::SetGains(const Gains& gains) { gains_ = gains; }

void PIDController::SetOutputLimits(float output_min, float output_max) {
  output_min_ = output_min;
  output_max_ = output_max;
  const float span = output_max_ - output_min_;
  integrator_ = Clamp(integrator_, -span, span);
}

void PIDController::Reset() {
  integrator_ = 0.0f;
  prev_error_ = 0.0f;
  initialized_ = false;
}

float PIDController::Update(float setpoint, float measurement,
                            float dt_seconds) {
  const float error = setpoint - measurement;

  // Seed the derivative on the first call so it doesn't spike from a stale
  // prev_error_ of zero.
  if (!initialized_) {
    prev_error_ = error;
    initialized_ = true;
  }

  const float p_term = gains_.kp * error;

  // Compute a *candidate* integrator value; whether we commit to it depends on
  // the anti-windup check below.
  const float candidate_integrator = integrator_ + gains_.ki * error * dt_seconds;

  const float d_term = gains_.kd * (error - prev_error_) / dt_seconds;

  // Unsaturated effort using the candidate integrator.
  const float unsaturated = p_term + candidate_integrator + d_term;

  // Conditional integration: only accept the new integrator unless we are
  // already past a rail and the error would drive us further past it.
  const bool winding_up =
      (unsaturated > output_max_ && error > 0.0f) ||
      (unsaturated < output_min_ && error < 0.0f);
  if (!winding_up) {
    integrator_ = candidate_integrator;
  }

  // Hard safety clamp on the integrator itself, bounded by the output span so
  // it can never dominate beyond the achievable range.
  const float span = output_max_ - output_min_;
  integrator_ = Clamp(integrator_, -span, span);

  const float output = Clamp(p_term + integrator_ + d_term, output_min_,
                             output_max_);

  prev_error_ = error;
  return output;
}
