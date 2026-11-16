// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <deque>
#include <gtest/gtest.h>
#include "tai_robot_one/terminal_braking.hpp"
#include "tai_robot_one/path_tracking.hpp"

namespace
{
// Include measured-command delay and drive acceleration rather than an
// instantaneous ideal chassis. Angles mirror the latest real corrections.
double runTurn(double degrees, double delay, bool predictive)
{
  const double dt = .01;
  const double target = degrees * tai_robot_one::kPi / 180.;
  double yaw = 0., speed = 0., command = 0.;
  bool braking = false, complete = false;
  int quiet = 0;
  std::deque<double> commands(static_cast<size_t>(std::round(delay / dt)) + 1, 0.);
  for (int i = 0; i < 1600; ++i) {
    if (i % 5 == 0 && !complete) {
      const double remaining = target - yaw;
      const double tolerance = (predictive ? 2. : 1.) * tai_robot_one::kPi / 180.;
      if (std::abs(remaining) <= tolerance && (!predictive || std::abs(speed) <= .02)) {
        complete = true;
        command = 0.;
      } else if (predictive && braking) {
        command = 0.;
        quiet = std::abs(speed) <= .02 ? quiet + 1 : 0;
        if (quiet >= 4) {braking = false;}
      } else if (predictive && (std::abs(remaining) <= tolerance ||
        tai_robot_one::shouldBrakeTerminalTurn(remaining, speed, .25, 1.5)))
      {
        command = 0.;
        braking = true;
        quiet = 0;
      } else {
        command = std::copysign(.22, remaining);
      }
    }
    commands.push_back(command);
    const double delivered = commands.front(); commands.pop_front();
    const bool slowing = std::abs(delivered) < std::abs(speed) || delivered * speed < 0.;
    speed += std::clamp(delivered - speed, -(slowing ? 1.5 : .6) * dt,
      (slowing ? 1.5 : .6) * dt);
    yaw += speed * dt;
    if (complete && std::abs(speed) < 1e-9 && i > 300) {break;}
  }
  return (target - yaw) * 180. / tai_robot_one::kPi;
}
}  // namespace

TEST(TerminalBraking, DelayedDriveStopsNearTheTargetInsteadOfOscillatingAcrossIt)
{
  for (double target : {3.283, -2.149, 3.712, -3.014, 45., -45.}) {
    for (double delay : {.15, .20, .25}) {
      EXPECT_LE(std::abs(runTurn(target, delay, true)), 2.) << target << " / " << delay;
    }
  }
  EXPECT_GT(std::abs(runTurn(3.283, .20, false)), 2.);
}

TEST(TerminalBraking, DoesNotApplyAnEarlyBrakeWhileRotatingAwayFromTheTarget)
{
  EXPECT_FALSE(tai_robot_one::shouldBrakeTerminalTurn(.05, -.22, .20, 1.5));
  EXPECT_FALSE(tai_robot_one::shouldBrakeTerminalTurn(.05, 0., .20, 1.5));
  EXPECT_TRUE(tai_robot_one::shouldBrakeTerminalTurn(.05, .22, .20, 1.5));
}

TEST(TerminalBraking, NewSmallRearGoalIsOptInAndKeepsTightGeometricBounds)
{
  EXPECT_FALSE(tai_robot_one::terminalReverseCorrectionAllowed(-.052, -.015, .054, .03, .024, false));
  EXPECT_TRUE(tai_robot_one::terminalReverseCorrectionAllowed(-.052, -.015, .054, .03, .024, false, true));
  EXPECT_FALSE(tai_robot_one::terminalReverseCorrectionAllowed(-.13, 0., .13, .03, 0., false, true));
  EXPECT_FALSE(tai_robot_one::terminalReverseCorrectionAllowed(-.05, .04, .07, .03, 0., false, true));
  EXPECT_FALSE(tai_robot_one::terminalReverseCorrectionAllowed(-.05, 0., .05, .03, .5, false, true));
}
