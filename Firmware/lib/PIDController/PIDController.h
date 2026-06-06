// PIDController.h — A small, dependency-free PID controller.
//
// This controller is intentionally decoupled from any particular sensor or
// actuator. It operates purely on floats (setpoint, measurement, dt) and
// returns a control effort clamped to a configurable output range, so it can
// be reused for any single-input/single-output loop: heater duty cycle, motor
// speed, valve position, etc.
//
// Features:
//   * Configurable output limits (e.g. 0..1 for a duty cycle).
//   * Conditional-integration anti-windup: the integrator stops accumulating
//     when the output is saturated and the error would push it further into
//     saturation. This prevents the long overshoot that a naive integrator
//     produces after a large step.
//   * Caller-supplied dt, so timing is owned by the control loop, not the
//     controller. This keeps the class testable off-target.
//
// Usage:
//   PIDController pid({.kp = 0.2f, .ki = 0.0025f, .kd = 0.0f}, 0.0f, 1.0f);
//   float effort = pid.update(setpoint, measurement, dtSeconds);

#pragma once

class PIDController {
 public:
  struct Gains {
    float kp;
    float ki;
    float kd;
  };

  // Construct with gains and the inclusive output range the controller is
  // allowed to command (defaults suit a 0..1 duty cycle).
  PIDController(const Gains& gains, float output_min = 0.0f,
                float output_max = 1.0f);

  // Retune at runtime. Existing integrator state is preserved.
  void SetGains(const Gains& gains);

  // Change the clamp applied to both the output and the integrator. The
  // integrator is re-clamped immediately to stay within the new span.
  void SetOutputLimits(float output_min, float output_max);

  // Clear all internal state (integrator, previous error). Call this whenever
  // the loop has been paused or the plant has been disturbed out-of-band.
  void Reset();

  // Advance the controller by one step.
  //   setpoint     desired process value
  //   measurement  current process value
  //   dt_seconds   elapsed time since the previous call (must be > 0)
  // Returns the control effort clamped to [output_min, output_max].
  float Update(float setpoint, float measurement, float dt_seconds);

  // Introspection (useful for telemetry / tuning).
  float integrator() const { return integrator_; }
  float last_error() const { return prev_error_; }

 private:
  Gains gains_;
  float output_min_;
  float output_max_;

  float integrator_;
  float prev_error_;
  bool initialized_;
};
