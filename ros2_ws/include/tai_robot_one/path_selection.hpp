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
// Compare length first (0.1 mm numerical buckets), then heading variation
// and stop-to-turn manoeuvres. A smoother long detour cannot beat a shorter
// route. Buckets give a strict ordering, unlike pairwise epsilon comparisons.
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
  constexpr double numerical_resolution = 0.0001;
  const double turns = 0.10 * turning + 0.05 * stops;
  return std::floor(length / numerical_resolution + 0.5) * numerical_resolution +
         numerical_resolution * 0.99 * (turns / (1.0 + turns));
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_SELECTION_HPP_
