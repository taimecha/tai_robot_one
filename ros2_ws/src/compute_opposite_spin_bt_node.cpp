// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0

#include <atomic>
#include <cmath>
#include <memory>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tai_robot_one
{

class ComputeOppositeSpin final : public BT::SyncActionNode
{
public:
  ComputeOppositeSpin(
    const std::string & name,
    const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    std::string cmd_vel_topic{"/cmd_vel_smoothed"};
    (void)getInput("cmd_vel_topic", cmd_vel_topic);
    subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        constexpr double kMinimumTurnRate = 0.02;
        constexpr double kMinimumForwardSpeed = 0.03;

        // Once normal forward motion resumes, the next recovery may learn a
        // new failed turn direction.  Until then, keep the chosen escape
        // direction latched so our own recovery spin cannot make the next
        // recovery reverse back toward the obstacle.
        if (msg->linear.x > kMinimumForwardSpeed) {
          recovery_turn_sign_.store(0, std::memory_order_relaxed);
        }

        if (recovery_turn_sign_.load(std::memory_order_relaxed) != 0) {
          return;
        }

        if (msg->angular.z > kMinimumTurnRate) {
          last_turn_sign_.store(1, std::memory_order_relaxed);
        } else if (msg->angular.z < -kMinimumTurnRate) {
          last_turn_sign_.store(-1, std::memory_order_relaxed);
        }
      });
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("recovery_angle", 0.7853981634, "Absolute recovery angle"),
      BT::InputPort<std::string>(
        "cmd_vel_topic", "/cmd_vel_smoothed", "Topic used to remember turn direction"),
      BT::OutputPort<double>("spin_dist", "Recovery angle opposite to the previous turn")
    };
  }

  BT::NodeStatus tick() override
  {
    double recovery_angle = 0.7853981634;
    (void)getInput("recovery_angle", recovery_angle);
    recovery_angle = std::abs(recovery_angle);

    const int previous_sign = last_turn_sign_.load(std::memory_order_relaxed);
    int recovery_sign = recovery_turn_sign_.load(std::memory_order_relaxed);
    if (recovery_sign == 0) {
      recovery_sign = previous_sign > 0 ? -1 : 1;
      recovery_turn_sign_.store(recovery_sign, std::memory_order_relaxed);
    }
    const double opposite_angle = recovery_sign * recovery_angle;
    setOutput("spin_dist", opposite_angle);

    RCLCPP_INFO(
      node_->get_logger(),
      "Recovery turn: previous direction=%s, trying %.1f deg in the opposite direction",
      previous_sign > 0 ? "left" : (previous_sign < 0 ? "right" : "unknown"),
      opposite_angle * 180.0 / 3.14159265358979323846);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  std::atomic<int> last_turn_sign_{0};
  std::atomic<int> recovery_turn_sign_{0};
};

}  // namespace tai_robot_one

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tai_robot_one::ComputeOppositeSpin>("ComputeOppositeSpin");
}
