// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__PATH_SELECTION_HPP_
#define TAI_ROBOT_ONE__PATH_SELECTION_HPP_
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>
#include "tai_robot_one/path_tracking.hpp"
namespace tai_robot_one
{
// Measure the actual incoming direction 25 cm before the end, ignoring a
// one-cell kink that a loaded robot cannot track on approach.
inline double terminalPivotAngle(const std::vector<TrackingPose> & path)
{
  if (path.size() < 2) {return std::numeric_limits<double>::infinity();}
  double final_travel_heading = path.back().yaw;
  double terminal_prefix = 0.0;
  for (size_t i = path.size() - 1; i > 0; --i) {
    const double dx = path[i].x - path[i - 1].x;
    const double dy = path[i].y - path[i - 1].y;
    const double segment = std::hypot(dx, dy);
    if (terminal_prefix + segment >= 0.25 || i == 1) {
      const double fraction = segment > 1e-9 ?
        std::min(1.0, (0.25 - terminal_prefix) / segment) : 0.0;
      const double gx = path.back().x - (path[i].x - fraction * dx);
      const double gy = path.back().y - (path[i].y - fraction * dy);
      if (std::hypot(gx, gy) >= 0.005) {
        final_travel_heading = std::atan2(gy, gx);
      }
      break;
    }
    terminal_prefix += segment;
  }
  return std::abs(wrapAngle(path.back().yaw - final_travel_heading));
}
// Prefer the shortest collision-checked route. A terminal pivot has a small
// tie-breaking cost, but must not send the robot around a visible clear gap.
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
  const double wheel_travel_cost = 0.01 * std::max(0.0, terminalPivotAngle(path) - 0.5);
  constexpr double numerical_resolution = 0.0001;
  const double turns = 0.10 * turning + 0.05 * stops;
  return std::floor((length + wheel_travel_cost) / numerical_resolution + 0.5) *
         numerical_resolution +
         numerical_resolution * 0.99 * (turns / (1.0 + turns));
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_SELECTION_HPP_
