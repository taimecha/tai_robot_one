// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "tai_robot_one/rotation_sweep.hpp"

using tai_robot_one::chooseRotation;
using tai_robot_one::kPi;
using tai_robot_one::rotationSweepClear;
using tai_robot_one::wrapAngle;

TEST(RotationSweep, PrefersShortSafeDirection)
{
  EXPECT_DOUBLE_EQ(*chooseRotation(1.0, [](double) {return true;}), 1.0);
  EXPECT_DOUBLE_EQ(*chooseRotation(-1.0, [](double) {return true;}), -1.0);
}

TEST(RotationSweep, OppositeMeansOppositePreviousSignNotAlwaysLeft)
{
  EXPECT_NEAR(*chooseRotation(1.0, [](double a) {return a < 0;}), 1.0-2*kPi, 1e-12);
  EXPECT_NEAR(*chooseRotation(-1.0, [](double a) {return a > 0;}), 2*kPi-1.0, 1e-12);
}

TEST(RotationSweep, RejectsWhenBothDirectionsBlocked)
{
  EXPECT_FALSE(chooseRotation(1.0, [](double) {return false;}));
}

TEST(RotationSweep, ChecksIntermediateAndFinalOrientations)
{
  EXPECT_FALSE(rotationSweepClear(0, 1, [](double a) {return a > .49 && a < .51;}));
  EXPECT_FALSE(rotationSweepClear(0, -1, [](double a) {return a <= -1;}));
  EXPECT_FALSE(rotationSweepClear(0, 1, [](double a) {return a == 0;}));
  EXPECT_TRUE(rotationSweepClear(0, 1, [](double) {return false;}));
}

TEST(RotationSweep, LongRotationDoesNotReverseAtPiBoundary)
{
  double remaining = -4.0;
  remaining -= wrapAngle(3.1 - (-3.1));
  EXPECT_LT(remaining, -3.8);
  EXPECT_GT(remaining, -4.0);
}

TEST(RotationSweep, SamplesBelowHalfOfTwoCentimeterCellAtForkTip)
{
  double previous = 0.0;
  double maximum_step = 0.0;
  EXPECT_TRUE(rotationSweepClear(0.0, 0.1, [&](double angle) {
    maximum_step = std::max(maximum_step, angle - previous);
    previous = angle;
    return false;
  }));
  EXPECT_LT(0.84 * maximum_step, 0.01);
}
