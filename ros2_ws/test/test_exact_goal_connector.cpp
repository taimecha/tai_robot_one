// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <vector>

#include "gtest/gtest.h"
#include "tai_robot_one/exact_goal_connector.hpp"

namespace
{
geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x; p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0); p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

std::vector<geometry_msgs::msg::Point> rectangle(double back = -.30, double front = .78,
  double half_width = .25)
{
  std::vector<geometry_msgs::msg::Point> result(4);
  result[0].x = back; result[0].y = -half_width;
  result[1].x = front; result[1].y = -half_width;
  result[2].x = front; result[2].y = half_width;
  result[3].x = back; result[3].y = half_width;
  return result;
}

nav_msgs::msg::Path pathTo(const geometry_msgs::msg::PoseStamped & endpoint)
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  path.poses = {pose(endpoint.pose.position.x - .10, endpoint.pose.position.y), endpoint};
  return path;
}

void obstacle(nav2_costmap_2d::Costmap2D & map, double x, double y,
  unsigned char cost = nav2_costmap_2d::LETHAL_OBSTACLE)
{
  unsigned int mx, my;
  ASSERT_TRUE(map.worldToMap(x, y, mx, my));
  map.setCost(mx, my, cost);
}

TEST(ExactGoalConnector, NativeFiveCentimeterFalseSuccessNowTargetsStoredA)
{
  nav2_costmap_2d::Costmap2D map(232, 178, .05, -4.066, -3.243, 0);
  const auto start = pose(3.00459417016134, 1.424157162286861, -1.5429036278867065);
  const auto goal = pose(3.051, 1.457, -1.5734143206728881);
  auto path = pathTo(pose(3.034, 1.457, -1.5734143206728881));
  EXPECT_LT(std::hypot(start.pose.position.x - path.poses.back().pose.position.x,
    start.pose.position.y - path.poses.back().pose.position.y), .05);
  EXPECT_GT(std::hypot(start.pose.position.x - goal.pose.position.x,
    start.pose.position.y - goal.pose.position.y), .05);
  const auto original = path.poses;
  tai_robot_one::appendExactGoal(path, start, goal, map, rectangle(), [] {return false;});
  ASSERT_EQ(path.poses.size(), original.size() + 1);
  EXPECT_EQ(path.poses[0], original[0]); EXPECT_EQ(path.poses[1], original[1]);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.x, goal.pose.position.x);
  EXPECT_DOUBLE_EQ(path.poses.back().pose.position.y, goal.pose.position.y);
  EXPECT_EQ(path.poses.back().pose.orientation, goal.pose.orientation);
}

TEST(ExactGoalConnector, ExistingExactEndpointIsReplacedWithoutDuplicate)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  auto goal = pose(.01, .02, .3); auto path = pathTo(goal);
  const auto size = path.poses.size();
  tai_robot_one::appendExactGoal(path, pose(-.1, .02), goal, map, rectangle(), [] {return false;});
  EXPECT_EQ(path.poses.size(), size); EXPECT_EQ(path.poses.back().pose, goal.pose);
}

TEST(ExactGoalConnector, RejectsObstacleEntirelyInsidePaddedFootprint)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  obstacle(map, .3, .01);
  auto path = pathTo(pose(0, 0)); const auto before = path;
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), pose(.05, 0),
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
  EXPECT_EQ(path, before);
}

TEST(ExactGoalConnector, ChecksIntermediateRotationWithLongForks)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  obstacle(map, .45, .40);
  const auto footprint = rectangle();
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, 0, 0, 0));
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, .02, 0, tai_robot_one::kPi / 2));
  auto path = pathTo(pose(0, 0));
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0),
    pose(.02, 0, tai_robot_one::kPi / 2), map, footprint, [] {return false;}),
    nav2_core::NoValidPathCouldBeFound);
}

TEST(ExactGoalConnector, ChecksIntermediateTranslation)
{
  nav2_costmap_2d::Costmap2D map(500, 500, .005, -1.0, -1.0, 0);
  const auto footprint = rectangle(-.005, .005, .005);
  obstacle(map, .04, 0);
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, 0, 0, 0));
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, .08, 0, 0));
  auto path = pathTo(pose(0, 0));
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), pose(.08, 0),
    map, footprint, [] {return false;}), nav2_core::NoValidPathCouldBeFound);
}

TEST(ExactGoalConnector, ChecksConnectorBearingEvenWhenStartAndGoalYawMatch)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  obstacle(map, 0, .65);
  const auto footprint = rectangle();
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, 0, 0, 0));
  EXPECT_TRUE(tai_robot_one::exactGoalFootprintClear(map, footprint, 0, .05, 0));
  auto path = pathTo(pose(0, 0));
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), pose(0, .05),
    map, footprint, [] {return false;}), nav2_core::NoValidPathCouldBeFound);
}

TEST(ExactGoalConnector, RejectsUnknownAndMapBoundary)
{
  nav2_costmap_2d::Costmap2D map(100, 100, .02, -1.0, -1.0, 0);
  obstacle(map, .3, .01, nav2_costmap_2d::NO_INFORMATION);
  auto path = pathTo(pose(0, 0));
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), pose(.05, 0),
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
  map.resetMap(0, 0, 100, 100);
  path = pathTo(pose(.5, 0));
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(.4, 0), pose(.55, 0),
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
}

TEST(ExactGoalConnector, RejectsLargeConnectorInvalidFramesAndNonfiniteGoal)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  auto path = pathTo(pose(0, 0)); auto goal = pose(.101, 0);
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), goal,
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
  goal = pose(.05, 0); goal.header.frame_id = "odom";
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), goal,
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
  goal = pose(std::numeric_limits<double>::quiet_NaN(), 0);
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), goal,
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
  goal = pose(.05, 0); goal.pose.orientation.w = 0;
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), goal,
    map, rectangle(), [] {return false;}), nav2_core::NoValidPathCouldBeFound);
}

TEST(ExactGoalConnector, CancelIsHonoredDuringSweepWithoutChangingPath)
{
  nav2_costmap_2d::Costmap2D map(200, 200, .02, -2.0, -2.0, 0);
  auto path = pathTo(pose(0, 0)); const auto before = path;
  int calls = 0;
  EXPECT_THROW(tai_robot_one::appendExactGoal(path, pose(-.1, 0), pose(.08, 0, 1.5),
    map, rectangle(), [&calls] {return ++calls > 8;}), nav2_core::PlannerCancelled);
  EXPECT_GT(calls, 8); EXPECT_EQ(path, before);
}
}  // namespace
