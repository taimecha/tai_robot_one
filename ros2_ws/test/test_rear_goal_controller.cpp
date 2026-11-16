// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <cstdlib>
#include <gtest/gtest.h>
#include "../src/safe_rotation_rpp.cpp"

class ArrivalGoalChecker : public nav2_core::GoalChecker
{
public:
  void initialize(const rclcpp_lifecycle::LifecycleNode::WeakPtr &, const std::string &,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS>) override {}
  void reset() override {}
  bool isGoalReached(const geometry_msgs::msg::Pose &, const geometry_msgs::msg::Pose &,
    const geometry_msgs::msg::Twist &) override {return false;}
  bool getTolerances(geometry_msgs::msg::Pose & p, geometry_msgs::msg::Twist & v) override
  {
    p.position.x = p.position.y = .03;
    p.orientation.z = std::sin(2. * tai_robot_one::kPi / 360.);
    p.orientation.w = std::cos(2. * tai_robot_one::kPi / 360.);
    v.linear.x = .015; v.angular.z = .02;
    return true;
  }
};

TEST(RearGoalController, PrecisionArrivalBrakesBeforeTheTargetAndDoesNotTurnAroundForFiveCm)
{
  setenv("ROS_DOMAIN_ID", "231", 1);
  rclcpp::init(0, nullptr);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("precision_arrival_map");
  map->set_parameters({rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "map"), rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("resolution", .025), rclcpp::Parameter("width", 6), rclcpp::Parameter("height", 6),
    rclcpp::Parameter("footprint", "[[0.81,0.13],[0.81,-0.13],[0.41,-0.28],[-0.36,-0.28],[-0.36,0.28],[0.41,0.28]]")});
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("precision_arrival_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  node->declare_parameter("FollowPath.predictive_terminal_braking", true);
  node->declare_parameter("FollowPath.terminal_command_latency", .25);
  node->declare_parameter("FollowPath.allow_short_rear_goal_correction", true);
  node->declare_parameter("FollowPath.allow_reverse_route_tracking", false);
  node->declare_parameter("FollowPath.terminal_reverse_speed", .10);
  tai_robot_one::SafeRotationRPP controller;
  controller.configure(node, "FollowPath", tf, map); controller.activate();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map"; pose.pose.position.x = pose.pose.position.y = 2.;
  auto update = [&](double yaw) {
      pose.header.stamp = node->now();
      pose.pose.orientation.z = std::sin(yaw / 2.); pose.pose.orientation.w = std::cos(yaw / 2.);
      geometry_msgs::msg::TransformStamped t; t.header = pose.header; t.child_frame_id = "base_link";
      t.transform.translation.x = t.transform.translation.y = 2.; t.transform.rotation = pose.pose.orientation;
      tf->setTransform(t, "synthetic", false);
    };
  auto route = [&](double x, double y, double yaw) {
      nav_msgs::msg::Path path; path.header = pose.header;
      path.poses.push_back(pose);
      auto goal = pose; goal.pose.position.x = x; goal.pose.position.y = y;
      goal.pose.orientation.z = std::sin(yaw / 2.); goal.pose.orientation.w = std::cos(yaw / 2.);
      path.poses.push_back(goal); return path;
    };
  ArrivalGoalChecker checker;
  geometry_msgs::msg::Twist velocity;
  update(0.); controller.setPlan(route(2., 2., 4. * tai_robot_one::kPi / 180.));
  EXPECT_GT(controller.computeVelocityCommands(pose, velocity, &checker).twist.angular.z, 0.);
  update(tai_robot_one::kPi / 180.); velocity.angular.z = .22;
  EXPECT_DOUBLE_EQ(controller.computeVelocityCommands(pose, velocity, &checker).twist.angular.z, 0.);
  // A later pose sample must not restart the turn while the base is braking.
  update(1.5 * tai_robot_one::kPi / 180.);
  EXPECT_DOUBLE_EQ(controller.computeVelocityCommands(pose, velocity, &checker).twist.angular.z, 0.);
  controller.reset(); update(0.); velocity = geometry_msgs::msg::Twist{};
  controller.setPlan(route(1.948, 1.985, .024));
  auto command = controller.computeVelocityCommands(pose, velocity, &checker);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0.);
  EXPECT_NEAR(command.twist.linear.x, -.10, 1e-9);
  unsigned int mx, my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(1.60, 2., mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(pose, velocity, &checker), nav2_core::NoValidControl);
  controller.deactivate(); controller.cleanup(); map->on_cleanup(rclcpp_lifecycle::State());
  map.reset(); node.reset(); tf.reset(); rclcpp::shutdown();
}

TEST(RearGoalController, ReversesWithoutHalfTurnStopsAtToleranceAndChecksRearCollision)
{
  setenv("ROS_DOMAIN_ID", "231", 1);
  rclcpp::init(0, nullptr);
  auto map = std::make_shared<nav2_costmap_2d::Costmap2DROS>("reverse_controller_map");
  map->set_parameters({
    rclcpp::Parameter("plugins", std::vector<std::string>{}),
    rclcpp::Parameter("global_frame", "map"),
    rclcpp::Parameter("robot_base_frame", "base_link"),
    rclcpp::Parameter("resolution", .025),
    rclcpp::Parameter("footprint", "[[0.81,0.13],[0.81,-0.13],[0.41,-0.28],[-0.36,-0.28],[-0.36,0.28],[0.41,0.28]]"),
    rclcpp::Parameter("width", 6), rclcpp::Parameter("height", 6)});
  ASSERT_EQ(map->on_configure(rclcpp_lifecycle::State()), nav2_util::CallbackReturn::SUCCESS);
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("reverse_controller_test");
  auto tf = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf->setUsingDedicatedThread(true);
  node->declare_parameter("FollowPath.desired_linear_vel", .28);
  node->declare_parameter("FollowPath.approach_velocity_scaling_dist", .30);
  node->declare_parameter("FollowPath.min_approach_linear_velocity", .10);
  tai_robot_one::SafeRotationRPP controller;
  controller.configure(node, "FollowPath", tf, map);
  controller.activate();
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  auto update = [&](double x, double y, double yaw) {
      pose.header.stamp = node->now();
      pose.pose.position.x = x; pose.pose.position.y = y;
      pose.pose.orientation.z = std::sin(yaw / 2);
      pose.pose.orientation.w = std::cos(yaw / 2);
      geometry_msgs::msg::TransformStamped transform;
      transform.header = pose.header;
      transform.child_frame_id = "base_link";
      transform.transform.translation.x = x; transform.transform.translation.y = y;
      transform.transform.rotation = pose.pose.orientation;
      tf->setTransform(transform, "synthetic", false);
    };
  auto route = [](double start, double end) {
      nav_msgs::msg::Path path;
      path.header.frame_id = "map";
      for (int i = 0; i <= 40; ++i) {
        geometry_msgs::msg::PoseStamped point;
        point.header = path.header;
        point.pose.position.x = start + (end - start) * i / 40;
        point.pose.position.y = 2;
        point.pose.orientation.w = 1;
        path.poses.push_back(point);
      }
      return path;
    };
  update(2, 2, 0);
  controller.setPlan(route(2, 1));
  geometry_msgs::msg::Twist velocity;
  auto command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_LT(command.twist.linear.x, 0);
  EXPECT_GE(command.twist.linear.x, -.08);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0);
  update(1.06, 2, 0);
  velocity.linear.x = -.08;
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_LT(command.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0);
  update(1.04, 2, 0);
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0);
  // Brake before switching from backward arrival to an ordinary forward goal.
  controller.setPlan(route(1.04, 2.04));
  EXPECT_DOUBLE_EQ(controller.computeVelocityCommands(pose, velocity, nullptr).twist.linear.x, 0);
  velocity.linear.x = 0;
  EXPECT_GT(controller.computeVelocityCommands(pose, velocity, nullptr).twist.linear.x, 0);
  // A rear curve steers while backing, without first rotating 180 degrees.
  nav_msgs::msg::Path curve;
  curve.header.frame_id = "map";
  for (const auto & p : std::vector<tai_robot_one::TrackingPose>{{2,2,0}, {1.95,2.0025,-.10},
      {1.8,2.0275,-.28}, {1.5,2.13,-.35}, {1.35,2.194,-.12}, {1.3,2.2,0}})
  {
    geometry_msgs::msg::PoseStamped point;
    point.header = curve.header;
    point.pose.position.x = p.x; point.pose.position.y = p.y;
    point.pose.orientation.z = std::sin(p.yaw/2);
    point.pose.orientation.w = std::cos(p.yaw/2);
    curve.poses.push_back(point);
  }
  update(2, 2, 0);
  controller.setPlan(curve);
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_LT(command.twist.linear.x, 0);
  EXPECT_LT(command.twist.angular.z, 0);
  // An obstacle just behind the current rear edge must stop reverse tracking.
  update(2, 2, 0);
  controller.setPlan(route(2, 1));
  unsigned int mx, my;
  ASSERT_TRUE(map->getCostmap()->worldToMap(1.60, 2, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  EXPECT_THROW(controller.computeVelocityCommands(pose, velocity, nullptr), nav2_core::NoValidControl);
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  // At the first accepted XY, a nearby cell blocks the yaw sweep. Drive the
  // last few centimetres straight to the clear endpoint before pivoting.
  auto final_straight = route(2.0, 2.9);
  final_straight.poses.back().pose.orientation.z = std::sin(1.2 / 2);
  final_straight.poses.back().pose.orientation.w = std::cos(1.2 / 2);
  update(2.86, 2, 0);
  controller.setPlan(final_straight);
  ASSERT_TRUE(map->getCostmap()->worldToMap(2.8625, 2.6125, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  velocity.linear.x = 0;
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_GT(command.twist.linear.x, 0);
  EXPECT_DOUBLE_EQ(command.twist.angular.z, 0);
  update(2.89, 2, 0);
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_DOUBLE_EQ(command.twist.linear.x, 0);
  EXPECT_GT(command.twist.angular.z, 0);
  // The shorter terminal rotation hits a newly observed fork obstacle. The
  // opposite complete sweep remains clear and must stay committed each tick.
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::FREE_SPACE);
  nav_msgs::msg::Path turn;
  turn.header.frame_id = "map";
  geometry_msgs::msg::PoseStamped at;
  at.pose.position.x = 2;
  at.pose.position.y = 2;
  at.pose.orientation.w = 1;
  turn.poses.push_back(at);
  at.pose.position.x = 2.03;
  at.pose.orientation.z = std::sin(2.4 / 2);
  at.pose.orientation.w = std::cos(2.4 / 2);
  turn.poses.push_back(at);
  ASSERT_TRUE(map->getCostmap()->worldToMap(2.0, 2.70, mx, my));
  map->getCostmap()->setCost(mx, my, nav2_costmap_2d::LETHAL_OBSTACLE);
  update(2, 2, 0);
  controller.setPlan(turn);
  velocity = geometry_msgs::msg::Twist{};
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_LT(command.twist.angular.z, 0);
  update(2, 2, -.60);
  command = controller.computeVelocityCommands(pose, velocity, nullptr);
  EXPECT_LT(command.twist.angular.z, 0);
  controller.deactivate(); controller.cleanup();
  map->on_cleanup(rclcpp_lifecycle::State());
  map.reset(); node.reset(); tf.reset();
  rclcpp::shutdown();
}
