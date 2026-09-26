// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__PATH_SELECTION_HPP_
#define TAI_ROBOT_ONE__PATH_SELECTION_HPP_
#include <cmath>
#include <limits>
#include <vector>
#include "tai_robot_one/path_tracking.hpp"
namespace tai_robot_one
{
// Length dominates; penalize heading variation and repeated stop-to-turn
// manoeuvres without smoothing a safe path into an unchecked obstacle.
inline double routePreference(const std::vector<TrackingPose> & path)
{
  if (path.size() < 2) {return std::numeric_limits<double>::infinity();}
  double length = 0.0, turning = 0.0;
  unsigned stops = 0;
  bool stationary_turn = false;
  for (size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y) ||
      !std::isfinite(path[i].yaw)) {return std::numeric_limits<double>::infinity();}
    if (i == 0) {continue;}
    const double d = std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
    const double a = std::abs(wrapAngle(path[i].yaw - path[i - 1].yaw));
    const bool turn = d < 0.005 && a > 0.02;
    if (turn && !stationary_turn) {++stops;}
    stationary_turn = turn;
    length += d;
    turning += a;
  }
  return length + 0.10 * turning + 0.05 * stops;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_SELECTION_HPP_
