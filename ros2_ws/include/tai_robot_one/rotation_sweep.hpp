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
std::optional<double> chooseRotation(double angle, SweepPredicate clear)
{
  angle = wrapAngle(angle);
  if (clear(angle)) {
    return angle;
  }
  const double alternative = angle > 0.0 ? angle - 2.0 * kPi : angle + 2.0 * kPi;
  if (clear(alternative)) {
    return alternative;
  }
  return std::nullopt;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__ROTATION_SWEEP_HPP_
