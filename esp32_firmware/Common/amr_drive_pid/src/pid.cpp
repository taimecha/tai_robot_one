#include "pid.hpp"

#include <math.h>

namespace amr_drive {
namespace {

float clampFloat(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

}  // namespace

VelocityPid::VelocityPid()
    : integral_term_(0.0f),
      previous_measurement_rpm_(0.0f),
      previous_output_pwm_(0),
      initialized_(false) {}

int16_t VelocityPid::update(float target_rpm,
                            float measured_rpm,
                            const PidConfig &config) {
  if (!isfinite(target_rpm) || !isfinite(measured_rpm)) {
    reset();
    return 0;
  }

  const int16_t output_limit =
      config.output_limit > 0 ? config.output_limit : 255;
  const float integral_limit =
      config.integral_limit > 0.0f ? config.integral_limit : output_limit;
  const float error_rpm = target_rpm - measured_rpm;

  // Keep the original controller's conditional-integration behavior so gains
  // obtained with the separate web tuner transfer directly to production.
  if (abs(previous_output_pwm_) < output_limit) {
    integral_term_ += config.ki_per_sample * error_rpm;
    integral_term_ =
        clampFloat(integral_term_, -integral_limit, integral_limit);
  }

  float derivative_measurement = 0.0f;
  if (initialized_) {
    derivative_measurement = measured_rpm - previous_measurement_rpm_;
  }

  float feedforward_pwm = 0.0f;
  if (target_rpm > 0.5f) {
    feedforward_pwm =
        config.feedforward_gain * target_rpm +
        config.feedforward_deadband_pwm;
  } else if (target_rpm < -0.5f) {
    feedforward_pwm =
        config.feedforward_gain * target_rpm -
        config.feedforward_deadband_pwm;
  }

  const float requested_pwm =
      feedforward_pwm + config.kp * error_rpm + integral_term_ -
      config.kd_per_sample * derivative_measurement;
  const float limited_pwm =
      clampFloat(requested_pwm, -output_limit, output_limit);

  previous_measurement_rpm_ = measured_rpm;
  previous_output_pwm_ = static_cast<int16_t>(limited_pwm);
  initialized_ = true;
  return previous_output_pwm_;
}

void VelocityPid::reset(float measured_rpm) {
  integral_term_ = 0.0f;
  previous_measurement_rpm_ = measured_rpm;
  previous_output_pwm_ = 0;
  initialized_ = true;
}

float VelocityPid::integralTerm() const {
  return integral_term_;
}

}  // namespace amr_drive
