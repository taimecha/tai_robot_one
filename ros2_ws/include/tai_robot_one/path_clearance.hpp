// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__PATH_CLEARANCE_HPP_
#define TAI_ROBOT_ONE__PATH_CLEARANCE_HPP_

#include <algorithm>
#include <cmath>
#include "tai_robot_one/rotation_sweep.hpp"

namespace tai_robot_one
{
// Sample both translation and yaw: the fork tip must not jump over a cell.
template<typename CollisionPredicate>
bool poseSegmentClear(
  double x0, double y0, double a0, double x1, double y1, double a1,
  double resolution, CollisionPredicate collision)
{
  const double angle = wrapAngle(a1 - a0);
  const double step = std::min(0.01, resolution * 0.5);
  if (step <= 0.0) {return false;}
  const int samples = std::max(1, static_cast<int>(std::ceil(
    (std::hypot(x1 - x0, y1 - y0) + 0.9 * std::abs(angle)) / step)));
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / samples;
    if (collision(x0 + t * (x1 - x0), y0 + t * (y1 - y0), a0 + t * angle)) {
      return false;
    }
  }
  return true;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_CLEARANCE_HPP_
