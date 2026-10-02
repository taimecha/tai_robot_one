// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include "tai_robot_one/rotation_sweep.hpp"
#include "tai_robot_one/path_clearance.hpp"
#include "tai_robot_one/path_tracking.hpp"
#include "tai_robot_one/path_selection.hpp"
#include "tai_robot_one/escape_memory.hpp"

TEST(EscapeMemory, SmallAdjustmentsInSameRegionCommitRetreat)
{
  tai_robot_one::EscapeMemory memory;
  memory.noteAdjustment(0, 0);
  EXPECT_FALSE(memory.committed);
  memory.noteAdjustment(.15, 0);
  EXPECT_TRUE(memory.committed);
  EXPECT_FALSE(memory.observe(.60, 0));  // advancing is not backing out
  memory.beginReverse(.15, 0, 0);
  EXPECT_FALSE(memory.observe(.05, 0));
  EXPECT_FALSE(memory.observe(.15, .40));  // sideways distance is not retreat
  EXPECT_FALSE(memory.observe(-.16, 0));
  EXPECT_FALSE(memory.observe(-1.50, 0));
  EXPECT_TRUE(memory.committed);
  memory.reset();  // a checked exit or new goal explicitly releases retreat
  EXPECT_FALSE(memory.committed);
  EXPECT_EQ(memory.attempts, 0u);
}

TEST(EscapeMemory, CollisionCommitIsIdempotentAndUsesActualReverseHeading)
{
  tai_robot_one::EscapeMemory memory;
  memory.commit();
  memory.beginReverse(1, 1, tai_robot_one::kPi / 2);
  memory.commit();
  memory.beginReverse(1, .90, 0);  // repeated guard must not reset the anchor
  EXPECT_FALSE(memory.observe(1, .80));
  EXPECT_FALSE(memory.observe(1, .69));
  EXPECT_FALSE(memory.observe(1, -1));
  EXPECT_TRUE(memory.committed);
  EXPECT_DOUBLE_EQ(memory.reverse_x, 1);
  EXPECT_DOUBLE_EQ(memory.reverse_y, 1);
  EXPECT_DOUBLE_EQ(memory.reverse_yaw, tai_robot_one::kPi / 2);
}

TEST(EscapeMemory, SignificantNewRegionAndNewGoalResetLocalAttempts)
{
  tai_robot_one::EscapeMemory memory;
  memory.noteAdjustment(0, 0);
  EXPECT_TRUE(memory.observe(.46, 0));
  memory.noteAdjustment(.46, 0);
  EXPECT_FALSE(memory.committed);
  memory.commit();
  memory.reset();
  EXPECT_FALSE(memory.committed);
  EXPECT_FALSE(memory.region_active);
}

TEST(PathPreference, PrefersShortRoutesAndLessTurning)
{
  using tai_robot_one::routePreference;
  EXPECT_LT(routePreference({{0, 0, 0}, {1, 0, 0}}),
    routePreference({{0, 0, 0}, {2, 0, 0}}));
  EXPECT_LT(routePreference({{0, 0, 0}, {1, 0, 0}}),
    routePreference({{0, 0, 0}, {0, 0, 1}, {1, 0, 1}, {1, 0, 0}}));
  EXPECT_LT(routePreference({{0, 0, 0}, {0, 0, 1}, {1, 0, 1}}),
    routePreference({{0, 0, 0}, {2, 0, 0}}));
}

TEST(PathPreference, ShorterTurningRouteBeatsSlightlyLongerSmoothArc)
{
  // Earlier weighted penalties would send the vehicle around a long loop.
  EXPECT_LT(tai_robot_one::routePreference({{0, 0, 0}, {0, 0, 3}, {1, 0, 3}, {1, 0, 0}}),
    tai_robot_one::routePreference({{0, 0, 0}, {1.001, 0, 0}}));
}

TEST(PathPreference, ShortDirectRouteBeatsLongTerminalApproach)
{
  const auto short_with_big_pivot = tai_robot_one::routePreference({
    {0, 0, -2.9}, {-2.01, -0.5, -2.9}, {-2.0, -0.5, 0}});
  const auto aligned_with_small_detour = tai_robot_one::routePreference({
    {0, 0, -2.9}, {-2.2, -0.5, -2.9}, {-2.0, -0.5, 0}});
  EXPECT_LT(short_with_big_pivot, aligned_with_small_detour);
}

TEST(PathPreference, RejectsEmptyOrNonFiniteRoutes)
{
  EXPECT_FALSE(std::isfinite(tai_robot_one::routePreference({})));
  EXPECT_FALSE(std::isfinite(tai_robot_one::routePreference(
    {{0, 0, 0}, {std::numeric_limits<double>::quiet_NaN(), 0, 0}})));
}

TEST(PathTracking, SmallCellOffsetDoesNotCauseLargeInitialSpin)
{
  const auto target = tai_robot_one::trackingTarget({
    {0.025, 0.025, 0.0}, {0.075, 0.025, 0.0}, {0.25, 0.0, 0.0}});
  EXPECT_NEAR(target.initial_angle, 0.0, 1e-9);
}

TEST(PathTracking, LocalizationShiftNearPlanStartKeepsOutgoingTangent)
{
  const auto target = tai_robot_one::trackingTarget({
    {-0.11, 0.06, 0.0}, {-0.06, 0.06, 0.0}, {0.20, 0.06, 0.0}});
  EXPECT_NEAR(target.initial_angle, 0.0, 1e-9);
}

TEST(PathTracking, LargeOffRouteOffsetStillUsesPointBearing)
{
  const auto target = tai_robot_one::trackingTarget({
    {0.3, 0.3, 0.0}, {0.4, 0.3, 0.0}});
  EXPECT_NEAR(target.initial_angle, tai_robot_one::kPi / 4, 1e-9);
}

using tai_robot_one::chooseRotation;
using tai_robot_one::rotationSweepClear;
using tai_robot_one::StationaryTurnGuard;
using tai_robot_one::wrapAngle;

TEST(RotationSweep, PrefersShortSafeDirection)
{
  EXPECT_DOUBLE_EQ(*chooseRotation(1.0, [](double) {return true;}), 1.0);
  EXPECT_DOUBLE_EQ(*chooseRotation(-1.0, [](double) {return true;}), -1.0);
}

TEST(RotationSweep, TriesOppositeCompleteSweepWhenShortSweepBlocked)
{
  EXPECT_NEAR(*chooseRotation(1.0, [](double a) {return a < 0;}),
    1.0 - 2.0 * tai_robot_one::kPi, 1e-9);
  EXPECT_NEAR(*chooseRotation(-1.0, [](double a) {return a > 0;}),
    -1.0 + 2.0 * tai_robot_one::kPi, 1e-9);
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

TEST(RotationSweep, StopsRepeatedSameDirectionTurnsAcrossYawWrap)
{
  StationaryTurnGuard guard;
  EXPECT_FALSE(guard.update(0.0, 0.0, 3.0));
  for (int i = 1; i <= 36; ++i) {
    EXPECT_FALSE(guard.update(0.0, 0.0, wrapAngle(3.0 + i * 0.18)));
  }
  EXPECT_TRUE(guard.update(0.0, 0.0, wrapAngle(3.0 + 37 * 0.18)));
}

TEST(RotationSweep, MeaningfulTranslationResetsTurnBudget)
{
  StationaryTurnGuard guard;
  EXPECT_FALSE(guard.update(0.0, 0.0, 0.0));
  EXPECT_FALSE(guard.update(0.0, 0.0, 3.0));
  EXPECT_FALSE(guard.update(0.05, 0.0, 3.0));
  EXPECT_FALSE(guard.update(0.05, 0.0, 3.5));
}

TEST(RotationSweep, ShortRetreatClearsTrippedBudget)
{
  StationaryTurnGuard guard;
  EXPECT_FALSE(guard.update(0, 0, 0));
  for (int i = 1; i <= 70; ++i) {
    guard.update(0, 0, wrapAngle(i * 0.1));
  }
  EXPECT_TRUE(guard.update(0, 0, wrapAngle(7.0)));
  EXPECT_FALSE(guard.update(-0.05, 0, wrapAngle(7.0)));
}

TEST(PathClearance, RejectsObstacleBetweenWaypoints)
{
  EXPECT_FALSE(tai_robot_one::poseSegmentClear(0, 0, 0, 1, 0, 0, 0.025,
    [](double x, double, double) {return x > 0.495 && x < 0.505;}));
}

TEST(PathClearance, RejectsForkSweepEvenWhenEndpointsFit)
{
  EXPECT_FALSE(tai_robot_one::poseSegmentClear(0, 0, 0, 0, 0, 1.0, 0.025,
    [](double, double, double a) {return a > 0.49 && a < 0.51;}));
}

TEST(PathClearance, AcceptsClearCorridor)
{
  EXPECT_TRUE(tai_robot_one::poseSegmentClear(0, 0, 0, 1, 0, 0, 0.025,
    [](double, double, double) {return false;}));
}

TEST(RotationSweep, FinalAlignmentNeverTakesLongDetour)
{
  EXPECT_FALSE(chooseRotation(0.12, [](double a) {return a < 0;}, false));
  EXPECT_NEAR(*chooseRotation(2 * tai_robot_one::kPi - 0.12,
    [](double) {return true;}, false), -0.12, 1e-9);
}

TEST(PathTracking, AdvancesBeforePlannedRightTurn)
{
  const auto target = tai_robot_one::trackingTarget({
    {0, 0, 0}, {0.05, 0, 0}, {0.10, 0, 0}, {0.15, 0, 0},
    {0.15, 0, -0.2}, {0.15, 0, -0.6}, {0.15, 0, -1.0},
    {0.18, -0.05, -1.0}, {0.25, -0.15, -1.0}});
  EXPECT_NEAR(target.initial_angle, 0, 1e-9);
  EXPECT_NEAR(target.lookahead_limit, 0.15, 1e-9);
  EXPECT_FALSE(target.at_turn);
}

TEST(PathTracking, RotatesOnlyAfterReachingPlannedTurnPoint)
{
  const auto target = tai_robot_one::trackingTarget({
    {0.01, 0, 0}, {0.01, 0, -0.2}, {0.01, 0, -1.0}, {0.15, -0.2, -1.0}});
  EXPECT_TRUE(target.at_turn);
  EXPECT_NEAR(target.planned_turn, -1.0, 1e-9);
}

TEST(PathTracking, ReleasesAlignedTurnToResumeTranslation)
{
  const auto target = tai_robot_one::trackingTarget({
    {0.01, 0, 1.0}, {0.01, 0, 0.2}, {0.01, 0, 0}, {0.2, 0, 0}});
  EXPECT_FALSE(target.at_turn);
  EXPECT_TRUE(std::isinf(target.lookahead_limit));
}

TEST(PathTracking, ReleasesAlignedTurnAfterSkidSteerPositionDrift)
{
  const auto target = tai_robot_one::trackingTarget({
    {0.03, 0, -1.0}, {0.03, 0, 0.0}, {0.30, 0, 0.0}});
  EXPECT_FALSE(target.at_turn);
  EXPECT_TRUE(std::isinf(target.lookahead_limit));
}

TEST(PathTracking, CompletedTurnBehindRobotUsesOutgoingTangent)
{
  const auto target = tai_robot_one::trackingTarget({
    {-.03, 0, 1.2}, {-.03, 0, .6}, {-.03, 0, 0},
    {.02, .003, 0}, {.12, .009, 0}});
  EXPECT_FALSE(target.at_turn);
  EXPECT_TRUE(std::isinf(target.lookahead_limit));
  EXPECT_NEAR(target.initial_angle, std::atan2(.003, .05), 1e-9);
}

TEST(PathTracking, ReleasedTurnWithLateralSkidDriftDoesNotStartAnotherSpin)
{
  const auto target = tai_robot_one::trackingTarget({
    {-.02, .04, 1.2}, {-.02, .04, .6}, {-.02, .04, 0},
    {.03, .04, 0}, {.08, .04, 0}});
  EXPECT_FALSE(target.at_turn);
  EXPECT_NEAR(target.initial_angle, 0, 1e-9);
}

TEST(PathTracking, ReleasedTurnDoesNotAimBackAtOldCuspBeforeNextTurn)
{
  const auto target = tai_robot_one::trackingTarget({
    {-.04, -.01, 1.0}, {-.04, -.01, 0}, {.20, -.01, 0},
    {.20, -.01, -.7}, {.30, -.12, -.7}});
  EXPECT_FALSE(target.at_turn);
  EXPECT_NEAR(target.initial_angle, 0, 1e-9);
  EXPECT_NEAR(target.lookahead_limit, std::hypot(.20, .01), 1e-9);
}

TEST(PathTracking, TerminalRetreatRequiresPriorArrivalAndAlignedShortCorridor)
{
  using tai_robot_one::terminalReverseCorrectionAllowed;
  EXPECT_TRUE(terminalReverseCorrectionAllowed(-.10, -.02, .03, .04, .08, true));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(-.10, -.02, .03, .04, .08, false));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(-.10, -.02, .30, .04, .08, true));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(-.20, -.02, .03, .04, .08, true));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(-.10, -.05, .03, .04, .08, true));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(-.10, -.02, .03, .04, .70, true));
  EXPECT_FALSE(terminalReverseCorrectionAllowed(.10, -.02, .03, .04, .08, true));
}
