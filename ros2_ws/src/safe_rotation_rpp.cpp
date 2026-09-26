// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <mutex>
#include <string>

#include <nav2_core/controller_exceptions.hpp>
#include <nav2_regulated_pure_pursuit_controller/regulated_pure_pursuit_controller.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tai_robot_one/rotation_sweep.hpp"
#include "tai_robot_one/path_tracking.hpp"

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
    // Keep a tripped turn guard across FollowPath retries for the same goal.
    Base::reset();
  }

  void setPlan(const nav_msgs::msg::Path & path) override
  {
    bool new_goal = false;
    if (!path.poses.empty()) {
      const auto & goal = path.poses.back().pose;
      const double goal_yaw = tf2::getYaw(goal.orientation);
      if (!have_goal_ || path.header.frame_id != goal_frame_ ||
        std::hypot(goal.position.x - goal_x_, goal.position.y - goal_y_) > 0.03 ||
        std::abs(wrapAngle(goal_yaw - goal_yaw_)) > 0.05)
      {
        new_goal = true;
        turn_guard_.reset();
        turn_limit_exceeded_ = false;
        goal_frame_ = path.header.frame_id;
        goal_x_ = goal.position.x;
        goal_y_ = goal.position.y;
        goal_yaw_ = goal_yaw;
        have_goal_ = true;
      }
    }
    // Replanning every two seconds must not interrupt and restart an
    // already-safe turn toward the same goal. That repeatedly changed the
    // heading target and led to back-and-forth turns near arrival.
    if (new_goal) {
      turning_ = false;
      has_reached_xy_tolerance_ = false;
    }
    // A blocked sweep may be retried only after a fresh global plan.
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
    turn_limit_exceeded_ = turn_guard_.update(
      pose.pose.position.x, pose.pose.position.y, yaw);
    if (turn_limit_exceeded_) {
      turning_ = false;
      turn_limit_exceeded_ = true;
      throw nav2_core::NoValidControl(
              "Stationary turn budget exceeded; a translated escape is required");
    }
    auto plan = path_handler_->transformGlobalPlan(pose, params_->max_robot_pose_search_dist);
    if (plan.poses.empty()) {
      throw nav2_core::InvalidPath("Empty transformed path");
    }
    global_path_pub_->publish(plan);
    geometry_msgs::msg::Pose tolerance;
    geometry_msgs::msg::Twist velocity_tolerance;
    double xy_tolerance = 0.05;
    double yaw_tolerance = 0.0872664626;
    if (goal_checker && goal_checker->getTolerances(tolerance, velocity_tolerance)) {
      xy_tolerance = tolerance.position.x;
      yaw_tolerance = std::abs(tf2::getYaw(tolerance.orientation));
    }
    goal_dist_tol_ = xy_tolerance;
    std::vector<TrackingPose> tracking_poses;
    for (const auto & p : plan.poses) {
      tracking_poses.push_back({p.pose.position.x, p.pose.position.y,
          tf2::getYaw(p.pose.orientation)});
    }
    const auto target = trackingTarget(tracking_poses);
    double angle = target.at_turn ? target.planned_turn : target.initial_angle;
    const auto & end = plan.poses.back().pose;
    const bool at_goal = std::hypot(end.position.x, end.position.y) < xy_tolerance;
    has_reached_xy_tolerance_ = has_reached_xy_tolerance_ || at_goal;
    if (has_reached_xy_tolerance_) {
      angle = wrapAngle(tf2::getYaw(end.orientation));
        // Arrival always tracks the current shortest signed yaw error.
        // A latched long sweep or a little overshoot must not cause a full lap.
      turning_ = false;
    }
    const double threshold = has_reached_xy_tolerance_ ? yaw_tolerance :
      (target.at_turn ? 0.04 : params_->rotate_to_heading_min_angle);
    if (!turning_) {
      if (std::abs(angle) > threshold) {
        const auto chosen = chooseRotation(angle,
            [&](double sweep) {return sweepClear(pose, yaw, sweep);},
          !has_reached_xy_tolerance_ && std::hypot(end.position.x, end.position.y) > 0.50);
        if (!chosen) {
          rotation_blocked_ = true;
          throw nav2_core::NoValidControl("Both complete heading sweeps blocked; replan required");
        }
        if (std::abs(*chosen) > StationaryTurnGuard::kMaxSweep) {
          turn_limit_exceeded_ = true;
          throw nav2_core::NoValidControl("Safe heading sweep exceeds stationary turn limit");
        }
        remaining_ = *chosen;
        turn_tolerance_ = has_reached_xy_tolerance_ ? yaw_tolerance : 0.04;
        turning_ = true;
        last_yaw_ = yaw;
      }
    } else {
      remaining_ -= wrapAngle(yaw - last_yaw_);
      last_yaw_ = yaw;
    }

    if (turning_) {
      std_msgs::msg::Bool rotating;
      rotating.data = true;
      is_rotating_to_heading_pub_->publish(rotating);
      if (std::abs(remaining_) < turn_tolerance_) {
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
      // Brake before a change of sign; acceleration limiting must never
      // turn a small requested right correction into another left command.
      if (remaining_ * velocity.angular.z < 0.0 && std::abs(velocity.angular.z) > 0.02) {
        return command;
      }
      double angular = std::copysign(
        std::min(params_->rotate_to_heading_angular_vel,
        std::sqrt(2.0 * params_->max_angular_accel * std::abs(remaining_))), remaining_);
      angular = std::clamp(angular,
        velocity.angular.z - params_->max_angular_accel * control_duration_,
        velocity.angular.z + params_->max_angular_accel * control_duration_);
      // Requested lower active-turn floor. Keep an exact zero at completion;
      // validate that 0.20 rad/s still overcomes loaded skid-steer friction.
      constexpr double kMinimumLoadedAngularSpeed = 0.20;
      angular = std::copysign(
        std::max(std::abs(angular), kMinimumLoadedAngularSpeed), angular);
      if (collision_checker_->isCollisionImminent(pose, 0.0, angular, 0.0)) {
        rotation_blocked_ = true;
        throw nav2_core::NoValidControl("Rotation command collision check failed");
      }
      command.twist.angular.z = angular;
      return command;
    }
    if (has_reached_xy_tolerance_) {
      geometry_msgs::msg::TwistStamped stop;
      stop.header = pose.header;
      return stop;
    }
    // Retain RPP's speed regulation, but follow the immediate segment rather
    // than letting Base demand an in-place turn toward a far-away carrot.
    // Stop lookahead at a planned rotate-in-place primitive. If the longer
    // tracking arc collides, try shorter carrots on this same validated path.
    std_msgs::msg::Bool rotating;
    rotating.data = false;
    is_rotating_to_heading_pub_->publish(rotating);
    double lookahead = std::min(getLookAheadDistance(velocity), target.lookahead_limit);
    lookahead = std::max(0.015, lookahead);
    while (true) {
      const auto carrot = getLookAheadPoint(lookahead, plan);
      const auto & p = carrot.pose.position;
      geometry_msgs::msg::PointStamped carrot_marker;
      carrot_marker.header = carrot.header;
      carrot_marker.point = p;
      carrot_pub_->publish(carrot_marker);
      const double d2 = p.x * p.x + p.y * p.y;
      const double curvature = d2 > 0.0001 ? 2.0 * p.y / d2 : 0.0;
      double linear = params_->desired_linear_vel;
      double sign = 1.0;
      applyConstraints(curvature, velocity,
        collision_checker_->costAtPose(pose.pose.position.x, pose.pose.position.y),
        plan, linear, sign);
      const double angular = linear * curvature;
      const double carrot_distance = std::hypot(p.x, p.y);
      if (!params_->use_collision_detection ||
        !collision_checker_->isCollisionImminent(pose, linear, angular, carrot_distance))
      {
        geometry_msgs::msg::TwistStamped command;
        command.header = pose.header;
        command.twist.linear.x = linear;
        command.twist.angular.z = angular;
        return command;
      }
      if (lookahead <= 0.05) {break;}
      lookahead = std::max(0.05, lookahead * 0.5);
    }
    throw nav2_core::NoValidControl("No collision-free tracking arc on the immediate path");
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
  bool turn_limit_exceeded_{false};
  bool have_goal_{false};
  std::string goal_frame_;
  double goal_x_{0.0};
  double goal_y_{0.0};
  double goal_yaw_{0.0};
  StationaryTurnGuard turn_guard_;
  double remaining_{0.0};
  double turn_tolerance_{0.04};
  double last_yaw_{0.0};
};
}  // namespace tai_robot_one

PLUGINLIB_EXPORT_CLASS(tai_robot_one::SafeRotationRPP, nav2_core::Controller)
