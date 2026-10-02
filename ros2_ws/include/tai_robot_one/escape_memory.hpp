// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#pragma once
#include <cmath>

namespace tai_robot_one
{
// Local recovery is not a complete route. Two small adjustments in the same
// odometry region are enough speculation before seeking different space.
// Rear travel, yaw changes, AMCL corrections and replans cannot release a
// committed retreat. A checked exit or a new goal releases it explicitly.
struct EscapeMemory
{
  bool region_active{false}, committed{false}, reversing{false};
  unsigned attempts{0};
  double region_x{0}, region_y{0}, reverse_x{0}, reverse_y{0}, reverse_yaw{0};

  void reset() {*this = EscapeMemory{};}
  bool observe(double x, double y)
  {
    if (!committed && region_active && std::hypot(x - region_x, y - region_y) > 0.45) {
      reset(); return true;
    }
    return false;
  }
  void noteAdjustment(double x, double y)
  {
    observe(x, y);
    if (!region_active) {region_active = true; region_x = x; region_y = y;}
    if (++attempts >= 2) {commit();}
  }
  void commit()
  {
    if (!committed) {committed = true; reversing = false;}
  }
  void beginReverse(double x, double y, double yaw)
  {
    if (committed && !reversing) {
      reversing = true; reverse_x = x; reverse_y = y; reverse_yaw = yaw;
    }
  }
};
}  // namespace tai_robot_one
