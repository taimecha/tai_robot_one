#pragma once

#include <stdint.h>

namespace amr_drive {

// Gains are expressed per control sample. Both firmware projects run this
// controller at a fixed 20 ms period so the values remain compatible with the
// original web-tuning firmware.
struct PidConfig {
  float kp;
  float ki_per_sample;
  float kd_per_sample;
  float feedforward_gain;
  int16_t feedforward_deadband_pwm;
  float integral_limit;
  int16_t output_limit;
};

class VelocityPid {
 public:
  VelocityPid();

  int16_t update(float target_rpm,
                 float measured_rpm,
                 const PidConfig &config);
  void reset(float measured_rpm = 0.0f);

  float integralTerm() const;

 private:
  float integral_term_;
  float previous_measurement_rpm_;
  int16_t previous_output_pwm_;
  bool initialized_;
};

}  // namespace amr_drive
