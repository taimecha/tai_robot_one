// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <memory>
#include <mutex>
#include <string>

#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tai_robot_one/exact_goal_connector.hpp"

namespace tai_robot_one
{
class ExactGoalSmac2D : public nav2_smac_planner::SmacPlanner2D
{
public:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    costmap_ros_ = costmap_ros;
    nav2_smac_planner::SmacPlanner2D::configure(parent, name, tf, costmap_ros);
  }

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker) override
  {
    const auto frame = costmap_ros_->getGlobalFrameID();
    for (const auto * pose : {&start, &goal}) {
      if (pose->header.frame_id != frame || !std::isfinite(pose->pose.position.x) ||
        !std::isfinite(pose->pose.position.y) || !std::isfinite(pose->pose.position.z))
      {
        throw nav2_core::NoValidPathCouldBeFound("Exact-goal planner requires finite poses in the global frame");
      }
      exactGoalYaw(pose->pose.orientation);
    }
    auto plan = nav2_smac_planner::SmacPlanner2D::createPlan(start, goal, cancel_checker);
    auto * map = costmap_ros_->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    appendExactGoal(plan, start, goal, *map, costmap_ros_->getRobotFootprint(), cancel_checker);
    return plan;
  }

  void cleanup() override
  {
    nav2_smac_planner::SmacPlanner2D::cleanup();
    costmap_ros_.reset();
  }

private:
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}  // namespace tai_robot_one

PLUGINLIB_EXPORT_CLASS(tai_robot_one::ExactGoalSmac2D, nav2_core::GlobalPlanner)
