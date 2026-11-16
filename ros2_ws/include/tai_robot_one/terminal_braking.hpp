// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
#define TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
#include <cmath>
namespace tai_robot_one
{
inline double angularStoppingTravel(double speed, double latency, double deceleration)
{
  if (!std::isfinite(speed) || !std::isfinite(latency) || !std::isfinite(deceleration) ||
    latency < 0.0 || deceleration <= 0.0)
  {
    return INFINITY;
  }
  return std::abs(speed) * latency + speed * speed / (2.0 * deceleration);
}

inline bool shouldBrakeTerminalTurn(
  double remaining, double speed, double latency, double deceleration)
{
  return std::isfinite(remaining) && std::isfinite(speed) &&
         remaining * speed > 0.0 && std::abs(speed) > 0.02 &&
         std::abs(remaining) <= angularStoppingTravel(speed, latency, deceleration);
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__TERMINAL_BRAKING_HPP_
