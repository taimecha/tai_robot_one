// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <mutex>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tai_robot_one/rotation_sweep.hpp"

namespace tai_robot_one
{
// RPP's translation regulation is retained. Heading alignment additionally
// checks the complete swept footprint, not just the next second of rotation.
class SafeRotationRPP : public
  nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController
{
  using Base = nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController;

public:
  void reset() override
  {
    turning_ = false;
    rotation_blocked_ = false;
    Base::reset();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    // A new plan invalidates the old alignment target. The next command
    // evaluates both full sweeps again before issuing any angular motion.
    turning_ = false;
    rotation_blocked_ = false;
    Base::setPlan(path);
  }

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    if (cancelling_) {
      turning_ = false;
      return Base::computeVelocityCommands(pose, velocity, goal_checker);
    }
    // A single clear costmap update must not restart an already-invalidated
    // turn and reset controller patience indefinitely. Require a new plan.
    if (rotation_blocked_) {
      throw nav2_core::NoValidControl("Rotation blocked on this plan; waiting for a new plan");
    }
    std::unique_lock<std::mutex> parameter_lock(param_handler_->getMutex());
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    const double yaw = tf2::getYaw(pose.pose.orientation);
    if (!turning_) {
      auto plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
      if (plan.poses.empty()) {
        throw nav2_core::InvalidPath("Empty transformed path");
      }
      geometry_msgs::msg::Pose tolerance;
      geometry_msgs::msg::Twist velocity_tolerance;
      double xy_tolerance = 0.15;
      if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
        xy_tolerance = tolerance.position.x;
      }
      const auto carrot = getLookAheadPoint(getLookAheadDistance(velocity), plan);
      double angle = std::atan2(carrot.pose.position.y, carrot.pose.position.x);
      const auto & end = plan.poses.back().pose;
      const bool at_goal = std::hypot(end.position.x, end.position.y) < xy_tolerance;
      if (!at_goal) {
        has_reached_xy_tolerance_ = false;
      }
      if (at_goal) {
        angle = tf2::getYaw(end.orientation);
      }
      const double threshold = at_goal ? 0.045 : params_->rotate_to_heading_min_angle;
      if (std::abs(angle) > threshold) {
        const auto chosen = chooseRotation(angle,
          [&](double sweep) {return sweepClear(pose, yaw, sweep);});
        if (!chosen) {
          rotation_blocked_ = true;
          throw nav2_core::NoValidControl("Both complete heading sweeps blocked; replan required");
        }
        remaining_ = *chosen;
        if (std::abs(remaining_) > kPi) {
          RCLCPP_INFO(logger_, "Short rotation blocked; committing to safe opposite sweep %.1f deg",
            remaining_ * 180.0 / kPi);
        }
        turning_ = true;
        last_yaw_ = yaw;
      }
    } else {
      remaining_ -= wrapAngle(yaw - last_yaw_);
      last_yaw_ = yaw;
    }

    if (turning_) {
      if (std::abs(remaining_) < 0.04) {
        turning_ = false;
        // Stop before handing translation back to RPP.
        geometry_msgs::msg::TwistStamped stop;
        stop.header = pose.header;
        return stop;
      }
      if (!sweepClear(pose, yaw, remaining_)) {
        // Do not silently change direction when fresh observations invalidate
        // the committed sweep. Stop and let planning choose another route.
        turning_ = false;
        rotation_blocked_ = true;
        throw nav2_core::NoValidControl("Committed rotation invalidated by updated costmap");
      }
      geometry_msgs::msg::TwistStamped command;
      command.header = pose.header;
      double angular = std::copysign(
        std::min(params_->rotate_to_heading_angular_vel,
        std::sqrt(2.0 * params_->max_angular_accel * std::abs(remaining_))), remaining_);
      angular = std::clamp(angular,
        velocity.angular.z - params_->max_angular_accel * control_duration_,
        velocity.angular.z + params_->max_angular_accel * control_duration_);
      // Bench and loaded-floor tests show that smaller commands cannot
      // reliably overcome four-wheel skid-steer scrub friction. Preserve an
      // exact zero at the completion threshold above, but keep an active turn
      // at or above the measured breakaway speed.
      constexpr double kMinimumLoadedAngularSpeed = 0.25;
      angular = std::copysign(
        std::max(std::abs(angular), kMinimumLoadedAngularSpeed), angular);
      if (collision_checker_->isCollisionImminent(pose, 0.0, angular, 0.0)) {
        rotation_blocked_ = true;
        throw nav2_core::NoValidControl("Rotation command collision check failed");
      }
      command.twist.angular.z = angular;
      return command;
    }
    lock.unlock();
    parameter_lock.unlock();
    return Base::computeVelocityCommands(pose, velocity, goal_checker);
  }

private:
  bool sweepClear(const geometry_msgs::msg::PoseStamped & pose, double yaw, double angle)
  {
    return rotationSweepClear(yaw, angle, [&](double heading) {
      return collision_checker_->inCollision(
        pose.pose.position.x, pose.pose.position.y, heading);
    });
  }
  bool turning_{false};
  bool rotation_blocked_{false};
  double remaining_{0.0};
  double last_yaw_{0.0};
};
}  // namespace tai_robot_one

PLUGINLIB_EXPORT_CLASS(tai_robot_one::SafeRotationRPP, nav2_core::Controller)
