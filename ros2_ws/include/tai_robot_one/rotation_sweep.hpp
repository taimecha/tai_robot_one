// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__ROTATION_SWEEP_HPP_
#define TAI_ROBOT_ONE__ROTATION_SWEEP_HPP_

#include <algorithm>
#include <cmath>
#include <optional>

namespace tai_robot_one
{
constexpr double kPi = 3.14159265358979323846;

inline double wrapAngle(double a)
{
  return std::atan2(std::sin(a), std::cos(a));
}

// A stationary turn should never consume multiple revolutions while chasing
// an unreachable path point. Replanning the same goal must not erase this
// budget; only meaningful translation or a genuinely new goal may reset it.
class StationaryTurnGuard
{
public:
  static constexpr double kMaxSweep = 2.1 * kPi;
  // A short, measured escape translation resets the budget.
  static constexpr double kResetTranslation = 0.04;

  void reset()
  {
    initialized_ = false;
    swept_ = 0.0;
  }

  bool update(double x, double y, double yaw)
  {
    if (!initialized_ || std::hypot(x - anchor_x_, y - anchor_y_) >= kResetTranslation) {
      anchor_x_ = x;
      anchor_y_ = y;
      last_yaw_ = yaw;
      swept_ = 0.0;
      initialized_ = true;
      return false;
    }
    swept_ += std::abs(wrapAngle(yaw - last_yaw_));
    last_yaw_ = yaw;
    return swept_ > kMaxSweep;
  }

private:
  bool initialized_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double last_yaw_{0.0};
  double swept_{0.0};
};

template<typename CollisionPredicate>
bool rotationSweepClear(double yaw, double angle, CollisionPredicate collision)
{
  // At the ~0.84 m padded fork radius, 0.01 rad is below half a 2 cm cell.
  const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(angle) / 0.01)));
  for (int i = 0; i <= steps; ++i) {
    if (collision(yaw + angle * static_cast<double>(i) / steps)) {
      return false;
    }
  }
  return true;
}

template<typename SweepPredicate>
std::optional<double> chooseRotation(double angle, SweepPredicate clear, bool allow_long = true)
{
  angle = wrapAngle(angle);
  if (clear(angle)) {
    return angle;
  }
  // Check the other complete swept footprint before resorting to reversing.
  const double alternate = angle > 0.0 ? angle - 2.0 * kPi : angle + 2.0 * kPi;
  if (allow_long && std::abs(angle) > 1e-6 && clear(alternate)) {
    return alternate;
  }
  return std::nullopt;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__ROTATION_SWEEP_HPP_
