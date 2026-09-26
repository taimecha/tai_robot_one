// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#ifndef TAI_ROBOT_ONE__PATH_TRACKING_HPP_
#define TAI_ROBOT_ONE__PATH_TRACKING_HPP_
#include <cmath>
#include <limits>
#include <vector>
#include "tai_robot_one/rotation_sweep.hpp"
namespace tai_robot_one
{
struct TrackingPose {double x, y, yaw;};
struct TrackingTarget
{
  double initial_angle{0.0};
  double lookahead_limit{std::numeric_limits<double>::infinity()};
  double planned_turn{0.0};
  bool at_turn{false};
};

inline TrackingTarget trackingTarget(const std::vector<TrackingPose> & path)
{
  TrackingTarget target;
  for (const auto & p : path) {
    if (std::hypot(p.x, p.y) >= 0.05) {
      target.initial_angle = std::atan2(p.y, p.x);
      break;
    }
  }
  // When already on the route, align with its first travel segment, not the
  // bearing to a tiny quantized cell offset. A 1--2 cm lateral offset at a
  // nearby 5 cm point can otherwise demand an unnecessary 30--40 deg spin.
  // Do not look past a rotate-in-place primitive before reaching its cusp.
  if (path.size() > 1 && std::hypot(path.front().x, path.front().y) <= 0.10) {
    for (size_t i = 1; i < path.size(); ++i) {
      const double dx = path[i].x - path[i - 1].x;
      const double dy = path[i].y - path[i - 1].y;
      if (std::hypot(dx, dy) >= 0.005) {
        target.initial_angle = std::atan2(dy, dx);
        break;
      }
      if (std::abs(wrapAngle(path[i].yaw - path[i - 1].yaw)) > 0.02) {break;}
    }
  }
  for (size_t i = 1; i < path.size(); ++i) {
    if (std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y) < 0.005 &&
      std::abs(wrapAngle(path[i].yaw - path[i - 1].yaw)) > 0.02)
    {
      size_t end = i;
      while (end + 1 < path.size() &&
        std::hypot(path[end + 1].x - path[i].x, path[end + 1].y - path[i].y) < 0.005)
      {
        ++end;
      }
      const double distance = std::hypot(path[i].x, path[i].y);
      // Already-aligned turn poses must not keep the vehicle stuck at a cusp.
      if (distance <= 0.015 && std::abs(wrapAngle(path[end].yaw)) <= 0.04) {
        i = end;
        continue;
      }
      target.lookahead_limit = distance;
      target.at_turn = distance <= 0.015;
      target.planned_turn = wrapAngle(path[end].yaw);
      break;
    }
  }
  return target;
}
}  // namespace tai_robot_one
#endif  // TAI_ROBOT_ONE__PATH_TRACKING_HPP_
