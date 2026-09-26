// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0
#include <atomic>
#include <chrono>
#include <memory>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/control_node.h>
#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/msg/costmap.hpp>
#include <nav2_msgs/action/spin.hpp>
#include <nav2_msgs/action/back_up.hpp>
#include <nav2_msgs/action/drive_on_heading.hpp>
#include <nav2_msgs/action/follow_path.hpp>
#include <nav2_costmap_2d/footprint_collision_checker.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include "tai_robot_one/path_clearance.hpp"
#include "tai_robot_one/path_selection.hpp"
#include "tai_robot_one/escape_memory.hpp"

namespace tai_robot_one
{
// One subscription/executor cache per tree, shared by all geometry queries.
// This also prevents an unvisited recovery condition starting with an empty
// private cache while other conditions already have valid observations.
struct ClearanceData
{
  explicit ClearanceData(const rclcpp::Node::SharedPtr & parent)
  {
    static std::atomic<unsigned> sequence{0};
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    options.parameter_overrides({rclcpp::Parameter(
      "use_sim_time", parent->get_parameter("use_sim_time").as_bool())});
    node = std::make_shared<rclcpp::Node>(
      "footprint_route_check_" + std::to_string(sequence++), options);
    executor.add_node(node);
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    global_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/global_costmap/costmap_raw", qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {global = msg;});
    local_sub = node->create_subscription<nav2_msgs::msg::Costmap>(
      "/local_costmap/costmap_raw", qos,
      [this](nav2_msgs::msg::Costmap::ConstSharedPtr msg) {local = msg;});
    footprint_sub = node->create_subscription<geometry_msgs::msg::PolygonStamped>(
      "/local_costmap/published_footprint", rclcpp::QoS(1).reliable(),
      [this](geometry_msgs::msg::PolygonStamped::ConstSharedPtr msg) {footprint = msg;});
  }
  rclcpp::Node::SharedPtr node;
  rclcpp::executors::SingleThreadedExecutor executor;
  nav2_msgs::msg::Costmap::ConstSharedPtr global, local;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint;
  // Shared across recovery conditions, but not carried into a new goal.
  bool turn_collision{false};
  double collision_min_retreat{0.15};
  bool failed_forward{false};
  geometry_msgs::msg::PoseStamped collision_goal;
  double collision_x{0.0}, collision_y{0.0}, collision_yaw{0.0};
  unsigned motion_revision{0};
  // A geometric end to a real retreat permits ONE stopped replan. It is
  // not proof of a dead end, and must not become a stationary retry loop.
  bool retreat_active{false}, retreat_stopped{false};
  geometry_msgs::msg::PoseStamped retreat_goal;
  double retreat_x{0.0}, retreat_y{0.0}, retreat_yaw{0.0};
  bool forward_active{false}, forward_geometric_stop{false};
  geometry_msgs::msg::PoseStamped forward_goal;
  double forward_x{0.0}, forward_y{0.0}, forward_yaw{0.0}, forward_distance{0.0};
  struct TurnAttempt {double x, y, heading;};
  std::vector<TurnAttempt> turn_attempts;
  geometry_msgs::msg::PoseStamped turn_goal;
  EscapeMemory escape_memory;
  bool turn_geometric_stop{false}, selected_turn_goal_facing{false}, turn_handoff{false};
  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr global_sub, local_sub;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr footprint_sub;
};

// Validate a candidate using the actual padded footprint, not a point or
// circumscribed circle. No commands are issued by this BT condition.
class PathFootprintClear final : public BT::ConditionNode
{
public:
  PathFootprintClear(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    auto parent = config.blackboard->get<rclcpp::Node::SharedPtr>("node");
    tf_ = config.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    if (!config.blackboard->get("tai_clearance_data", data_)) {
      data_ = std::make_shared<ClearanceData>(parent);
      config.blackboard->set("tai_clearance_data", data_);
    }
    node_ = data_->node;
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<bool>("data_only", false, "Check sensor/TF readiness without checking a path"),
      BT::InputPort<std::string>("motion", "path",
          "path/direct/forward/turn/turn_collision/reverse_needed/escape_available/"
          "recovery_allowed/replan_after_retreat/forward_candidate/forward_route/motion_fault_free/"
          "forward_begin/forward_safe/forward_complete/forward_collision/turn_begin/lost_exit"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
      BT::InputPort<double>("candidate_distance", 0.15, "Forward route search distance"),
      BT::InputPort<double>("candidate_angle", 0.0, "Selected checked recovery turn"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("candidate_start"),
      BT::InputPort<double>("local_check_distance", 1.0, "Radius requiring live local checks"),
      BT::InputPort<uint16_t>("planner_error", uint16_t{0}, "Planner result code"),
      BT::InputPort<uint16_t>("controller_error", uint16_t{0}, "Controller result code"),
      BT::InputPort<uint16_t>("motion_error", uint16_t{0}, "Motion action result code"),
      BT::InputPort<bool>("local_blocked", false, "Route failure is near the robot"),
      BT::InputPort<std::string>("planner_id", "SE2Fallback",
          "GridBased or SE2Fallback path semantics"),
      BT::InputPort<bool>("prepared", false, "Path already converted from Smac2D cell corners"),
      BT::OutputPort<nav_msgs::msg::Path>("checked_path"),
      BT::OutputPort<std::string>("path_planner"),
      BT::OutputPort<bool>("blocked_near_robot"),
      BT::OutputPort<double>("forward_distance"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("forward_start"),
      BT::OutputPort<double>("turn_angle")
    };
  }

  BT::NodeStatus tick() override
  {
    data_->executor.spin_some();
    global_ = data_->global;
    local_ = data_->local;
    footprint_ = data_->footprint;
    try {
      if (!global_ || !local_ || !footprint_ ||
        !fresh(global_->header.stamp, 3.0) || !fresh(local_->header.stamp, 1.0) ||
        !fresh(footprint_->header.stamp, 1.0))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Route check waiting for fresh costmaps and footprint");
        return BT::NodeStatus::FAILURE;
      }
      // AMCL's map->odom can be older while stationary. Use current odometry
      // composed with the latest localization, not the oldest common TF time.
      const auto odom_robot = tf_->lookupTransform(
        local_->header.frame_id, "base_footprint", tf2::TimePointZero);
      if (!fresh(odom_robot.header.stamp, 1.0)) {return BT::NodeStatus::FAILURE;}
      const auto odom_to_map = tf_->lookupTransform(
        global_->header.frame_id, local_->header.frame_id, tf2::TimePointZero);
      geometry_msgs::msg::PoseStamped odom_pose, map_pose;
      odom_pose.pose.position.x = odom_robot.transform.translation.x;
      odom_pose.pose.position.y = odom_robot.transform.translation.y;
      odom_pose.pose.orientation = odom_robot.transform.rotation;
      tf2::doTransform(odom_pose, map_pose, odom_to_map);
      const auto footprint_tf = tf_->lookupTransform(
        "base_footprint", footprint_->header.frame_id,
        rclcpp::Time(footprint_->header.stamp));
      nav2_costmap_2d::Footprint body;
      for (const auto & p : footprint_->polygon.points) {
        geometry_msgs::msg::PointStamped in, out;
        in.point.x = p.x;
        in.point.y = p.y;
        in.point.z = p.z;
        tf2::doTransform(in, out, footprint_tf);
        body.push_back(out.point);
      }
      if (body.size() < 3) {return BT::NodeStatus::FAILURE;}
      bool data_only = false;
      (void)getInput("data_only", data_only);
      if (data_only) {return BT::NodeStatus::SUCCESS;}
      const auto motion = getInput<std::string>("motion").value();
      const bool direct = motion == "direct";
      const bool forward_route = motion == "forward_route";
      const bool path_query = motion == "path" || direct || forward_route;
      const double requested_range = getInput<double>("local_check_distance").value();
      if (!std::isfinite(requested_range) || requested_range < 1.0 || requested_range > 1.30) {
        return BT::NodeStatus::FAILURE;
      }
      double local_range = requested_range;
      nav_msgs::msg::Path path;
      if ((motion == "path" || forward_route) &&
        (!getInput("path", path) || path.poses.empty() ||
        path.header.frame_id != global_->header.frame_id))
      {
        return BT::NodeStatus::FAILURE;
      }
      if (direct) {
        geometry_msgs::msg::PoseStamped goal;
        if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id) {
          return BT::NodeStatus::FAILURE;
        }
        const auto & start = map_pose.pose;
        const double dx = goal.pose.position.x - start.position.x;
        const double dy = goal.pose.position.y - start.position.y;
        const double distance = std::hypot(dx, dy);
        const double yaw = tf2::getYaw(start.orientation);
        const double heading = distance <= 0.05 ? yaw : std::atan2(dy, dx);
        if (!std::isfinite(distance) || distance > 0.75 ||
          std::abs(wrapAngle(heading - yaw)) > 0.10)
        {
          return BT::NodeStatus::FAILURE;
        }
        path.header = global_->header;
        auto first = map_pose;
        first.header = path.header;
        path.poses.push_back(first);
        if (std::abs(wrapAngle(heading - yaw)) > 1e-6) {
          auto aligned = first;
          aligned.pose.orientation.x = aligned.pose.orientation.y = 0.0;
          aligned.pose.orientation.z = std::sin(heading / 2);
          aligned.pose.orientation.w = std::cos(heading / 2);
          path.poses.push_back(aligned);
        }
        // Dense points keep pruning/initial alignment from treating the old
        // start point behind a moving vehicle as a new 180-degree target.
        const int samples = std::max(1, static_cast<int>(std::ceil(distance / 0.025)));
        for (int i = 1; i < samples; ++i) {
          auto point = first;
          const double t = static_cast<double>(i) / samples;
          point.pose.position.x += t * dx;
          point.pose.position.y += t * dy;
          point.pose.orientation.x = point.pose.orientation.y = 0.0;
          point.pose.orientation.z = std::sin(heading / 2);
          point.pose.orientation.w = std::cos(heading / 2);
          path.poses.push_back(point);
        }
        path.poses.push_back(goal);
      }
      if (path_query) {
        const auto planner_id = direct ? std::string("Direct") :
          getInput<std::string>("planner_id").value();
        if (planner_id == "GridBased" && !getInput<bool>("prepared").value()) {
          // Smac2D short paths can contain default/identity quaternions.
          // XY-only planning has no in-place orientation primitives: derive
          // travel headings from geometry, retaining the requested goal yaw.
          // Installed Jazzy Smac2D emits integer-cell corners. Use physical
          // cell centres, avoiding fake initial turns on short XY routes.
          const double half_cell = global_->metadata.resolution * 0.5;
          for (auto & p : path.poses) {
            p.pose.position.x += half_cell;
            p.pose.position.y += half_cell;
          }
          geometry_msgs::msg::PoseStamped goal;
          if (getInput("goal", goal) && goal.header.frame_id == path.header.frame_id) {
            // Acceptance must be measured against the requested goal, not
            // Smac2D's discretized map-cell corner (up to a cell away).
            // The complete validator below still rejects an unsafe endpoint.
            path.poses.back().pose = goal.pose;
          }
          for (size_t i = 0; i + 1 < path.poses.size(); ++i) {
            const auto & p = path.poses[i].pose.position;
            for (size_t j = i + 1; j < path.poses.size(); ++j) {
              const auto & next = path.poses[j].pose.position;
              if (std::hypot(next.x - p.x, next.y - p.y) < 1e-6) {continue;}
              const double a = std::atan2(next.y - p.y, next.x - p.x);
              auto & q = path.poses[i].pose.orientation;
              q.x = q.y = 0.0; q.z = std::sin(a / 2); q.w = std::cos(a / 2);
              break;
            }
          }
        }
        if (forward_route) {
          geometry_msgs::msg::PoseStamped candidate;
          geometry_msgs::msg::PoseStamped goal;
          if (!getInput("candidate_start", candidate) ||
            !getInput("goal", goal) || goal.header.frame_id != path.header.frame_id ||
            candidate.header.frame_id != path.header.frame_id ||
            std::hypot(path.poses.back().pose.position.x - goal.pose.position.x,
            path.poses.back().pose.position.y - goal.pose.position.y) > 0.075)
          {return BT::NodeStatus::FAILURE;}
          path.poses.back().pose = goal.pose;
          const auto & actual = map_pose.pose;
          const double yaw = tf2::getYaw(actual.orientation);
          const double dx = candidate.pose.position.x - actual.position.x;
          const double dy = candidate.pose.position.y - actual.position.y;
          const double advance = dx * std::cos(yaw) + dy * std::sin(yaw);
          const double lateral = -dx * std::sin(yaw) + dy * std::cos(yaw);
          if (!std::isfinite(advance) || advance < 0.10 || advance > 1.25 ||
            std::abs(lateral) > 0.02 ||
            std::abs(wrapAngle(tf2::getYaw(candidate.pose.orientation) - yaw)) > 0.05 ||
            std::hypot(path.poses.front().pose.position.x - candidate.pose.position.x,
            path.poses.front().pose.position.y - candidate.pose.position.y) > 0.075)
          {return BT::NodeStatus::FAILURE;}
          // Prepend the WHOLE actual forward departure, not just the route
          // starting at a hypothetical pocket. Keep lattice turn primitives.
          nav_msgs::msg::Path joined;
          joined.header = path.header;
          const int n = std::max(1, static_cast<int>(std::ceil(advance / 0.025)));
          for (int i = 0; i <= n; ++i) {
            auto p = map_pose;
            p.header = path.header;
            p.pose.position.x += dx * static_cast<double>(i) / n;
            p.pose.position.y += dy * static_cast<double>(i) / n;
            joined.poses.push_back(p);
          }
          // A quantized planner heading may need alignment AT the pocket,
          // never an unchecked coupled turn before reaching that location.
          auto aligned = candidate;
          aligned.header = path.header;
          aligned.pose.orientation = path.poses.front().pose.orientation;
          joined.poses.push_back(aligned);
          joined.poses.insert(joined.poses.end(), path.poses.begin(), path.poses.end());
          path = std::move(joined);
          local_range = std::max(1.0, advance + 0.05);
        }
        setOutput("checked_path", path);
        setOutput("path_planner", planner_id);
      }
      auto global_map = makeMap(*global_);
      auto local_map = makeMap(*local_);
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> gc(
        global_map.get()), lc(local_map.get());
      const auto to_local = tf_->lookupTransform(
        local_->header.frame_id, global_->header.frame_id, tf2::TimePointZero);
      const double tx = to_local.transform.translation.x;
      const double ty = to_local.transform.translation.y;
      const double ta = tf2::getYaw(to_local.transform.rotation);
      const double x = map_pose.pose.position.x;
      const double y = map_pose.pose.position.y;
      const double yaw = tf2::getYaw(map_pose.pose.orientation);
      auto collision = [&](double px, double py, double a) {
          if (occupied(gc, px, py, a, body)) {return true;}
          const double lx = tx + std::cos(ta) * px - std::sin(ta) * py;
          const double ly = ty + std::sin(ta) * px + std::cos(ta) * py;
          unsigned int mx, my;
          // Local observations are required for the launch sweep. Farther
          // along the route the global map checks the entire footprint.
          if (std::hypot(px - x, py - y) < local_range) {
            if (!local_map->worldToMap(lx, ly, mx, my)) {return true;}
            return occupied(lc, lx, ly, a + ta, body);
          }
          return false;
        };
      if (!path_query) {
        return checkManeuver(motion, x, y, yaw, ta, tx, ty, body, gc, lc);
      }
      setOutput("blocked_near_robot", false);
      // Trim the traversed prefix when revalidating a stable plan.
      size_t nearest = 0;
      double distance = std::numeric_limits<double>::infinity();
      for (size_t i = 0; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose.position;
        const double d = std::hypot(p.x - x, p.y - y);
        if (d < distance) {distance = d; nearest = i;}
      }
      const auto & first = path.poses[nearest].pose;
      const double first_yaw = tf2::getYaw(first.orientation);
      bool starts_forward = false;
      for (size_t i = nearest + 1; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose.position;
        if (std::hypot(p.x - x, p.y - y) >= 0.05) {
          starts_forward = std::abs(wrapAngle(std::atan2(p.y - y, p.x - x) - yaw)) < 0.35;
          break;
        }
        // A rotation primitive at the launch point requires an actual sweep.
        if (std::abs(wrapAngle(tf2::getYaw(path.poses[i].pose.orientation) - first_yaw)) >
          0.02)
        {
          break;
        }
      }
      if (!direct && starts_forward && !occupied(lc,
          tx + std::cos(ta) * x - std::sin(ta) * y,
          ty + std::sin(ta) * x + std::cos(ta) * y, yaw + ta, body))
      {
        // Allow departure from a raster overlap confined to the extra 3 cm
        // padding, only when the live local footprint is clear. Never erase
        // the core body, unknown cells, or any newly encountered obstacles.
        // These are private validation copies, not the actual costmaps.
        auto inner = body;
        nav2_costmap_2d::padFootprint(inner, -0.03);
        nav2_costmap_2d::Footprint outer_polygon, inner_polygon;
        nav2_costmap_2d::transformFootprint(x, y, yaw, body, outer_polygon);
        nav2_costmap_2d::transformFootprint(x, y, yaw, inner, inner_polygon);
        nav2_costmap_2d::Costmap2D outer_mask(
          global_map->getSizeInCellsX(), global_map->getSizeInCellsY(),
          global_map->getResolution(), global_map->getOriginX(), global_map->getOriginY());
        auto inner_mask = outer_mask;
        outer_mask.setConvexPolygonCost(outer_polygon, 1);
        inner_mask.setConvexPolygonCost(inner_polygon, 1);
        for (unsigned int iy = 0; iy < global_map->getSizeInCellsY(); ++iy) {
          for (unsigned int ix = 0; ix < global_map->getSizeInCellsX(); ++ix) {
            if (outer_mask.getCost(ix, iy) != 1 || inner_mask.getCost(ix, iy) == 1 ||
              global_map->getCost(ix, iy) != nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              continue;
            }
            double wx, wy;
            global_map->mapToWorld(ix, iy, wx, wy);
            const double lx = tx + std::cos(ta) * wx - std::sin(ta) * wy;
            const double ly = ty + std::sin(ta) * wx + std::cos(ta) * wy;
            unsigned int mx, my;
            if (local_map->worldToMap(lx, ly, mx, my) &&
              local_map->getCost(mx, my) < nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              global_map->setCost(ix, iy, nav2_costmap_2d::FREE_SPACE);
            }
          }
        }
      }
      if (!starts_forward && !chooseRotation(wrapAngle(first_yaw - yaw), [&](double sweep) {
          return rotationSweepClear(yaw, sweep, [&](double a) {
                   return collision(x, y, a);
          });
        }))
      {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
          "Initial footprint rotation blocked at (%.3f, %.3f), angle %.1f deg",
          x, y, wrapAngle(first_yaw - yaw) * 180.0 / kPi);
        setOutput("blocked_near_robot", true);
        return BT::NodeStatus::FAILURE;
      }
      const double resolution = std::min(
        global_map->getResolution(), local_map->getResolution());
      // Heading quantization (e.g. 99 deg robot vs 90 deg lattice) does not
      // require a launch spin when the controller can align while advancing.
      // Check that coupled motion from the real current yaw instead.
      double px = x, py = y, pa = starts_forward ? yaw : first_yaw;
      if (collision(x, y, yaw)) {
        setOutput("blocked_near_robot", true);
        return BT::NodeStatus::FAILURE;
      }
      const size_t begin = starts_forward && distance < 0.03 ? nearest + 1 : nearest;
      for (size_t i = begin; i < path.poses.size(); ++i) {
        const auto & p = path.poses[i].pose;
        const double a = tf2::getYaw(p.orientation);
        // RPP reaches the final XY first, then aligns the goal heading.
        // At arrival allow only the shortest terminal yaw correction.
        if (i + 1 == path.poses.size()) {
          if (!poseSegmentClear(
              px, py, pa, p.position.x, p.position.y, pa, resolution, collision) ||
            !chooseRotation(wrapAngle(a - pa), [&](double sweep) {
              return rotationSweepClear(pa, sweep, [&](double heading) {
                       return collision(p.position.x, p.position.y, heading);
              });
            }, false))
          {
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
              "Terminal footprint / short yaw correction blocked at (%.3f, %.3f)",
              p.position.x, p.position.y);
            setOutput("blocked_near_robot", std::hypot(p.position.x - x, p.position.y - y) < 1.0);
            return BT::NodeStatus::FAILURE;
          }
          break;
        }
        if (!poseSegmentClear(
            px, py, pa, p.position.x, p.position.y, a, resolution, collision))
        {
          RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
            "Path footprint blocked at pose %zu (%.3f, %.3f), yaw %.1f deg", i,
            p.position.x, p.position.y, a * 180.0 / kPi);
          setOutput("blocked_near_robot", std::hypot(p.position.x - x, p.position.y - y) < 1.0);
          return BT::NodeStatus::FAILURE;
        }
        px = p.position.x; py = p.position.y; pa = a;
      }
      return BT::NodeStatus::SUCCESS;
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 3000,
        "Footprint route check unavailable: %s", error.what());
      return BT::NodeStatus::FAILURE;
    }
  }

private:
  BT::NodeStatus checkManeuver(
    const std::string & mode, double x, double y, double yaw, double ta,
    double tx, double ty, const nav2_costmap_2d::Footprint & body,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & gc,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & lc)
  {
    geometry_msgs::msg::PoseStamped goal;
    if (!getInput("goal", goal) || goal.header.frame_id != global_->header.frame_id) {
      return BT::NodeStatus::FAILURE;
    }
    const auto same_goal = [&](const geometry_msgs::msg::PoseStamped & previous) {
        return previous.header.frame_id == goal.header.frame_id &&
          std::hypot(goal.pose.position.x - previous.pose.position.x,
          goal.pose.position.y - previous.pose.position.y) < 1e-6 &&
          std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
          tf2::getYaw(previous.pose.orientation))) < 1e-6;
      };
    if (!same_goal(data_->turn_goal)) {
      data_->turn_attempts.clear();
      data_->escape_memory.reset();
      data_->turn_geometric_stop = data_->selected_turn_goal_facing = false;
      data_->turn_handoff = false;
      data_->turn_goal = goal;
      ++data_->motion_revision;
    }
    if (!same_goal(data_->forward_goal)) {
      data_->forward_active = data_->forward_geometric_stop = false;
      data_->forward_goal = goal;
      ++data_->motion_revision;
    }
    if (data_->turn_collision &&
      (std::hypot(goal.pose.position.x - data_->collision_goal.pose.position.x,
      goal.pose.position.y - data_->collision_goal.pose.position.y) > 1e-6 ||
      std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
      tf2::getYaw(data_->collision_goal.pose.orientation))) > 1e-6))
    {
      data_->turn_collision = false;
      data_->failed_forward = false;
      ++data_->motion_revision;
    }
    if ((data_->retreat_active || data_->retreat_stopped) &&
      (std::hypot(goal.pose.position.x - data_->retreat_goal.pose.position.x,
      goal.pose.position.y - data_->retreat_goal.pose.position.y) > 1e-6 ||
      std::abs(wrapAngle(tf2::getYaw(goal.pose.orientation) -
      tf2::getYaw(data_->retreat_goal.pose.orientation))) > 1e-6))
    {
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
    }
    if (mode == "replan_after_retreat") {
      const auto error = getInput<uint16_t>("motion_error").value();
      const bool collision_stop = error == nav2_msgs::action::BackUp::Result::COLLISION_AHEAD;
      if ((!data_->retreat_stopped && !(data_->retreat_active && collision_stop)) ||
        (error != nav2_msgs::action::BackUp::Result::NONE && !collision_stop))
      {
        return BT::NodeStatus::FAILURE;
      }
      // Use odometry, not map localization: a pose correction or movement
      // toward the old dead end cannot authorize a new recovery cycle.
      const double ox = tx + std::cos(ta) * x - std::sin(ta) * y;
      const double oy = ty + std::sin(ta) * x + std::cos(ta) * y;
      const double retreat = -(ox - data_->retreat_x) * std::cos(data_->retreat_yaw) -
        (oy - data_->retreat_y) * std::sin(data_->retreat_yaw);
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
      if (retreat < 0.05) {return BT::NodeStatus::FAILURE;}
      RCLCPP_INFO(node_->get_logger(),
        "Retreat stopped after %.2f m measured travel; replan from the new pose before abort",
        retreat);
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "recovery_allowed" || mode == "motion_fault_free") {
      const auto planner = getInput<uint16_t>("planner_error").value();
      const auto controller = getInput<uint16_t>("controller_error").value();
      const bool near = getInput<bool>("local_blocked").value();
      // A distant blocked destination is not evidence that backing up here
      // will help. Nor should failed terminal yaw alignment move the goal XY.
      const bool healthy = controller == 0 || controller == 104 ||
        controller == 105 || controller == 106;
      if (mode == "motion_fault_free") {
        return healthy && std::hypot(goal.pose.position.x - x,
          goal.pose.position.y - y) > 0.10 ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
      const bool permitted = healthy && std::hypot(goal.pose.position.x - x,
          goal.pose.position.y - y) > 0.10 &&
        (planner == 205 || (planner == 0 && (near || controller == 104 ||
        controller == 105 || controller == 106)));
      return permitted ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    const double odom_x = tx + std::cos(ta) * x - std::sin(ta) * y;
    const double odom_y = ty + std::sin(ta) * x + std::cos(ta) * y;
    if (data_->escape_memory.observe(odom_x, odom_y)) {
      ++data_->motion_revision;
      RCLCPP_INFO(node_->get_logger(), "Escape reached a different odometry region; reconsider exits");
    }
    // Reverse guards run at BT frequency, but expensive swept-body queries
    // need only run at 5 Hz. Freshness/TF checks still run on EVERY tick.
    const auto now = std::chrono::steady_clock::now();
    if ((mode == "reverse_needed" || mode == "escape_available" || mode == "forward_safe") &&
      cached_mode_ == mode && cached_revision_ == data_->motion_revision &&
      now - cached_time_ < std::chrono::milliseconds(200))
    {
      return cached_status_;
    }
    auto local_collision = [&](double px, double py, double a) {
        return occupied(lc, tx + std::cos(ta) * px - std::sin(ta) * py,
                 ty + std::sin(ta) * px + std::cos(ta) * py, a + ta, body);
      };
    const double resolution = std::min(gc.getCostmap()->getResolution(),
      lc.getCostmap()->getResolution());
    auto forward_at = [&](double px, double py, double a, double distance, auto blocked) {
        return poseSegmentClear(px, py, a, px + distance * std::cos(a),
                 py + distance * std::sin(a), a, resolution, blocked);
      };
    // Start-occupied escape is translation OUT of an existing map overlap,
    // never a general static-map clearing. The unmodified endpoint must fit.
    // Any newly intersected global cell and every live local obstacle remain
    // blocking. No published costmap or physical footprint is changed.
    auto departure_map = *gc.getCostmap();
    nav2_costmap_2d::Footprint polygon;
    nav2_costmap_2d::transformFootprint(x, y, yaw, body, polygon);
    nav2_costmap_2d::Costmap2D mask(departure_map.getSizeInCellsX(),
      departure_map.getSizeInCellsY(), departure_map.getResolution(),
      departure_map.getOriginX(), departure_map.getOriginY());
    mask.setConvexPolygonCost(polygon, 1);
    if (!local_collision(x, y, yaw)) {
      for (unsigned int iy = 0; iy < departure_map.getSizeInCellsY(); ++iy) {
        for (unsigned int ix = 0; ix < departure_map.getSizeInCellsX(); ++ix) {
          if (mask.getCost(ix, iy) == 1 && departure_map.getCost(ix, iy) == 254) {
            departure_map.setCost(ix, iy, 0);
          }
        }
      }
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> dc(&departure_map);
    auto departure_collision = [&](double px, double py, double a) {
        return occupied(dc, px, py, a, body) || local_collision(px, py, a);
      };
    // Recovery must check live observations over the ENTIRE manoeuvre,
    // including a turning pocket more than one metre from the start.
    auto maneuver_collision = [&](double px, double py, double a) {
        return occupied(gc, px, py, a, body) || local_collision(px, py, a);
      };
    // A map-only contact in the EXTRA padding may be translated out of, but
    // never a core-body/local/unknown contact. Use private copies only. The
    // entire 30 cm release must fit and end clear on the unmodified maps.
    auto retreat_map = *gc.getCostmap();
    auto core = body;
    nav2_costmap_2d::padFootprint(core, -0.03);
    bool padding_release = false;
    if (occupied(gc, x, y, yaw, body) &&
      !local_collision(x, y, yaw) && !occupied(gc, x, y, yaw, core))
    {
      nav2_costmap_2d::Footprint core_polygon;
      nav2_costmap_2d::transformFootprint(x, y, yaw, core, core_polygon);
      auto core_mask = mask;
      core_mask.resetMap(0, 0, core_mask.getSizeInCellsX(), core_mask.getSizeInCellsY());
      core_mask.setConvexPolygonCost(core_polygon, 1);
      for (unsigned int iy = 0; iy < retreat_map.getSizeInCellsY(); ++iy) {
        for (unsigned int ix = 0; ix < retreat_map.getSizeInCellsX(); ++ix) {
          if (mask.getCost(ix, iy) == 1 && core_mask.getCost(ix, iy) != 1 &&
            retreat_map.getCost(ix, iy) == nav2_costmap_2d::LETHAL_OBSTACLE)
          {
            double wx, wy;
            retreat_map.mapToWorld(ix, iy, wx, wy);
            unsigned int lx, ly;
            if (lc.getCostmap()->worldToMap(
                tx + std::cos(ta) * wx - std::sin(ta) * wy,
                ty + std::sin(ta) * wx + std::cos(ta) * wy, lx, ly) &&
              lc.getCostmap()->getCost(lx, ly) < nav2_costmap_2d::LETHAL_OBSTACLE)
            {
              retreat_map.setCost(ix, iy, nav2_costmap_2d::FREE_SPACE);
              padding_release = true;
            }
          }
        }
      }
    }
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> rc(&retreat_map);
    auto retreat_collision = [&](double px, double py, double a) {
        return occupied(rc, px, py, a, body) || local_collision(px, py, a);
      };
    auto rear_clear = [&]() {
        if (forward_at(x, y, yaw, -0.16, maneuver_collision)) {return true;}
        return padding_release && forward_at(x, y, yaw, -0.30, retreat_collision) &&
          !maneuver_collision(x - 0.30 * std::cos(yaw), y - 0.30 * std::sin(yaw), yaw);
      };
    if (mode == "turn_collision") {
      // A geometry stop can be followed by another checked retreat. Never
      // turn a motor timeout, TF failure or unknown fault into reverse motion.
      if (getInput<uint16_t>("motion_error").value() !=
        nav2_msgs::action::Spin::Result::COLLISION_AHEAD)
      {
        return BT::NodeStatus::FAILURE;
      }
      data_->turn_collision = true;
      data_->collision_min_retreat = 0.15;
      data_->failed_forward = false;
      data_->collision_goal = goal;
      data_->collision_x = tx + std::cos(ta) * x - std::sin(ta) * y;
      data_->collision_y = ty + std::sin(ta) * x + std::cos(ta) * y;
      data_->collision_yaw = yaw + ta;
      ++data_->motion_revision;
      const bool rear = rear_clear();
      // Log the heading-exclusion policy, not a blanket retreat-distance rule.
      RCLCPP_WARN(node_->get_logger(),
        "Spin collision: %s; exclude attempted heading, check other exits before retreat",
        rear ? "rear checked, continuing escape" : "rear blocked, stopping safely");
      return rear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    // Measure signed rear travel in odom: an AMCL correction or just turning
    // at the same position must not count as having backed farther out.
    const double retreat = -(odom_x - data_->collision_x) * std::cos(data_->collision_yaw) -
      (odom_y - data_->collision_y) * std::sin(data_->collision_yaw);
    const bool recorded_turn_here = std::any_of(
      data_->turn_attempts.begin(), data_->turn_attempts.end(), [&](const auto & attempt) {
        return std::hypot(odom_x - attempt.x, odom_y - attempt.y) < 0.15;
      });
    // The attempted heading is excluded by useful_exit. Another fully checked
    // turn can execute immediately; only an unrecorded collision result needs
    // the conservative measured retreat before selecting a turn again.
    const bool force_retreat = data_->turn_collision && !recorded_turn_here &&
      retreat < data_->collision_min_retreat;
    if (mode == "forward_candidate") {
      const double d = getInput<double>("candidate_distance").value();
      const double goal_distance = std::hypot(goal.pose.position.x - x,
        goal.pose.position.y - y);
      if (force_retreat || data_->escape_memory.committed || !std::isfinite(goal_distance) ||
        !std::isfinite(d) || d < 0.15 || d > 1.20 ||
        d > goal_distance + 0.04 ||
        !forward_at(x, y, yaw, d, maneuver_collision)) {return BT::NodeStatus::FAILURE;}
      geometry_msgs::msg::PoseStamped start;
      start.header = global_->header;
      start.pose.position.x = x + d * std::cos(yaw);
      start.pose.position.y = y + d * std::sin(yaw);
      start.pose.orientation.z = std::sin(yaw / 2);
      start.pose.orientation.w = std::cos(yaw / 2);
      setOutput("forward_start", start);
      return BT::NodeStatus::SUCCESS;
    }
    // Turning pockets need extra room for raster/localization error and the
    // measured braking overrun. The actual translation/rear footprint is
    // unchanged; this margin is only for deciding it is time to stop backing.
    auto turning_body = body;
    nav2_costmap_2d::padFootprint(turning_body, 0.04);
    auto pocket_collision = [&](double px, double py, double a) {
        return occupied(gc, px, py, a, turning_body) || occupied(lc,
                 tx + std::cos(ta) * px - std::sin(ta) * py,
                 ty + std::sin(ta) * px + std::cos(ta) * py, a + ta, turning_body);
      };
    auto turn_fits = [&](double px, double py, double from, double angle) {
        const double sweep = std::abs(angle) < 1e-6 ? angle :
          angle + std::copysign(0.07, angle);
        return rotationSweepClear(from, sweep, [&](double a) {
                   return pocket_collision(px, py, a);
        });
      };
    struct Exit {bool clear{false}; double turn{0.0};};
    auto useful_exit = [&](double px, double py) -> Exit {
        if (maneuver_collision(px, py, yaw)) {return {};}
        const double dx = goal.pose.position.x - px;
        const double dy = goal.pose.position.y - py;
        const double distance = std::hypot(dx, dy);
        const double desired = wrapAngle(std::atan2(dy, dx) - yaw);
        // An open 15 cm ahead is NOT an exit if the same corridor ends at
        // a wall. Check a longer goal-directed corridor before stopping a
        // retreat. At an accepted XY only the terminal yaw is relevant.
        if (distance <= 0.05) {
          const double a = wrapAngle(tf2::getYaw(goal.pose.orientation) - yaw);
          return turn_fits(px, py, yaw, a) ? Exit{true, a} : Exit{};
        }
        if (!data_->escape_memory.committed && !(data_->failed_forward && retreat < 0.05) &&
          std::abs(desired) <= 0.10 && forward_at(
            px, py, yaw, std::min(0.75, distance), maneuver_collision))
        {
          return {true, 0.0};
        }
        std::vector<double> angles{desired};
        for (int i = 1; !data_->escape_memory.committed && i <= 12; ++i) {
          angles.push_back(i * kPi / 12);
          angles.push_back(-i * kPi / 12);
        }
        // Prefer a heading toward the goal, then the smaller signed sweep.
        // A tiny turn still pointing into the old dead end is not progress.
        std::sort(angles.begin(), angles.end(), [&](double a, double b) {
            return 2.0 * std::abs(wrapAngle(desired - a)) + std::abs(a) <
                   2.0 * std::abs(wrapAngle(desired - b)) + std::abs(b);
          });
        for (const double a : angles) {
          if (std::abs(a) < 0.10) {continue;}
          const double ox = tx + std::cos(ta) * px - std::sin(ta) * py;
          const double oy = ty + std::sin(ta) * px + std::cos(ta) * py;
          bool tried = false;
          for (const auto & attempt : data_->turn_attempts) {
            if (std::hypot(ox - attempt.x, oy - attempt.y) < 0.15 &&
              std::abs(wrapAngle(yaw + ta + a - attempt.heading)) < 0.12)
            {tried = true; break;}
          }
          if (tried) {continue;}
          if (!turn_fits(px, py, yaw, a) ||
            !forward_at(px, py, yaw + a, std::min(0.30, distance), pocket_collision))
          {continue;}
          // A safe angled corridor may be the first stage of a detour. Do
          // not require the still-blocked goal-facing turn at this position.
          // The next full route is planned/validated AFTER this stopped turn.
          return {true, a};
        }
        return {};
      };
    const auto current_exit = force_retreat ? Exit{} : useful_exit(x, y);
    if (mode == "lost_exit") {
      // Only a fresh geometric handoff failure permits another retreat.
      // A native Spin timeout/TF/unknown error must still stop the tree.
      if (getInput<uint16_t>("motion_error").value() != 0 ||
        (!data_->turn_handoff && !data_->turn_geometric_stop) ||
        (current_exit.clear && !data_->turn_geometric_stop) || !rear_clear())
      {return BT::NodeStatus::FAILURE;}
      data_->escape_memory.commit();
      data_->turn_geometric_stop = false;
      data_->turn_handoff = false;
      ++data_->motion_revision;
      RCLCPP_WARN(node_->get_logger(),
        "Turning pocket lost during braking; rear checked, continue seeking different space");
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_begin") {
      const double d = getInput<double>("candidate_distance").value();
      if (!std::isfinite(d) || d < 0.15 || d > 1.20) {
        return BT::NodeStatus::FAILURE;
      }
      if (force_retreat || data_->escape_memory.committed ||
        !forward_at(x, y, yaw, d, departure_collision) ||
        !useful_exit(x + d * std::cos(yaw), y + d * std::sin(yaw)).clear)
      {
        data_->forward_geometric_stop = true;
        return BT::NodeStatus::FAILURE;
      }
      data_->forward_active = true;
      data_->forward_geometric_stop = false;
      data_->forward_goal = goal;
      data_->forward_x = odom_x; data_->forward_y = odom_y;
      data_->forward_yaw = yaw + ta; data_->forward_distance = d;
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_safe") {
      if (!data_->forward_active) {return BT::NodeStatus::FAILURE;}
      const double travelled = (odom_x - data_->forward_x) * std::cos(data_->forward_yaw) +
        (odom_y - data_->forward_y) * std::sin(data_->forward_yaw);
      const double remaining = std::clamp(data_->forward_distance - travelled, 0.0, 1.20);
      const bool fits = !force_retreat &&
        forward_at(x, y, yaw, remaining, departure_collision) &&
        useful_exit(x + remaining * std::cos(yaw), y + remaining * std::sin(yaw)).clear;
      if (!fits) {
        data_->forward_geometric_stop = true;
        ++data_->motion_revision;
      }
      cached_mode_ = mode;
      cached_revision_ = data_->motion_revision;
      cached_time_ = now;
      cached_status_ = fits ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      return cached_status_;
    }
    if (mode == "forward_complete") {
      if (!data_->forward_active) {return BT::NodeStatus::FAILURE;}
      if (!current_exit.clear) {
        data_->forward_geometric_stop = true;
        ++data_->motion_revision;
        return BT::NodeStatus::FAILURE;
      }
      data_->forward_active = false;
      data_->escape_memory.noteAdjustment(odom_x, odom_y);
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "forward_collision") {
      const auto error = getInput<uint16_t>("motion_error").value();
      if ((error != 0 && error != nav2_msgs::action::DriveOnHeading::Result::COLLISION_AHEAD) ||
        (!data_->forward_geometric_stop &&
        error != nav2_msgs::action::DriveOnHeading::Result::COLLISION_AHEAD))
      {return BT::NodeStatus::FAILURE;}
      // Unlike an actually collided spin, a failed advance does not impose
      // an arbitrary retreat distance before accepting a NEW safe turn.
      data_->forward_active = data_->forward_geometric_stop = false;
      data_->escape_memory.commit();
      data_->turn_collision = true;
      data_->collision_min_retreat = 0.0;
      data_->failed_forward = true;
      data_->collision_goal = goal;
      data_->collision_x = odom_x; data_->collision_y = odom_y;
      data_->collision_yaw = yaw + ta;
      ++data_->motion_revision;
      const bool rear = rear_clear();
      RCLCPP_WARN(node_->get_logger(), "Forward escape blocked: %s",
        rear ? "checked rear corridor, switching to continuous retreat" : "rear blocked, stopping");
      return rear ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }
    double advance = 0.0;
    // Do not advance away from an already usable turn. Search actual turning
    // pockets, not arbitrary increments of clear floor. Every intervening
    // footprint and the destination turn/exit are checked before driving.
    if (mode == "forward" && !force_retreat && !data_->escape_memory.committed &&
      (!current_exit.clear || current_exit.turn == 0.0))
    {
      const double goal_distance = std::hypot(goal.pose.position.x - x,
        goal.pose.position.y - y);
      for (double d : {0.15, 0.30, 0.45, 0.60, 0.75, 0.90, 1.05, 1.20}) {
        if (d > goal_distance + 0.04) {continue;}
        if (!forward_at(x, y, yaw, d, departure_collision)) {break;}
        if (useful_exit(x + d * std::cos(yaw), y + d * std::sin(yaw)).clear) {
          advance = d; break;
        }
      }
    }
    if (mode == "forward") {
      if (advance == 0.0) {return BT::NodeStatus::FAILURE;}
      setOutput("forward_distance", advance);
      RCLCPP_INFO(node_->get_logger(),
        "Validated forward turning/exit pocket %.2f m; not reversing", advance);
      return BT::NodeStatus::SUCCESS;
    }
    const double turn = current_exit.turn;
    if (mode == "turn_begin") {
      const double a = getInput<double>("candidate_angle").value();
      data_->turn_geometric_stop = false;
      if (!std::isfinite(a) || std::abs(a) < 0.10 || std::abs(a) > kPi + 1e-6) {
        return BT::NodeStatus::FAILURE;
      }
      const double desired = wrapAngle(std::atan2(goal.pose.position.y - y,
        goal.pose.position.x - x) - yaw);
      if (force_retreat ||
        (data_->escape_memory.committed && std::abs(wrapAngle(desired - a)) > 0.10) ||
        !turn_fits(x, y, yaw, a) ||
        !forward_at(x, y, yaw + a, std::min(0.30,
        std::hypot(goal.pose.position.x - x, goal.pose.position.y - y)), pocket_collision))
      {data_->turn_geometric_stop = true; return BT::NodeStatus::FAILURE;}
      data_->selected_turn_goal_facing = std::abs(wrapAngle(desired - a)) <= 0.10;
      data_->escape_memory.noteAdjustment(odom_x, odom_y);
      data_->turn_attempts.push_back({odom_x, odom_y, wrapAngle(yaw + ta + a)});
      if (data_->turn_attempts.size() > 32) {
        data_->turn_attempts.erase(data_->turn_attempts.begin());
      }
      ++data_->motion_revision;
      return BT::NodeStatus::SUCCESS;
    }
    if (mode == "turn") {
      if (turn == 0.0) {return BT::NodeStatus::FAILURE;}
      setOutput("turn_angle", turn);
      return BT::NodeStatus::SUCCESS;
    }
    // While retreating, NEVER stop just because advancing back toward the
    // previous dead end is possible. Stop only at a useful exit HERE.
    const bool escape = current_exit.clear;
    const bool retreat_clear = !escape && rear_clear();
    if (mode == "reverse_needed") {
      if (retreat_clear && !data_->retreat_active) {
        data_->turn_handoff = false;
        data_->escape_memory.beginReverse(odom_x, odom_y, yaw + ta);
        data_->retreat_active = true;
        data_->retreat_stopped = false;
        data_->retreat_goal = goal;
        data_->retreat_x = odom_x;
        data_->retreat_y = odom_y;
        data_->retreat_yaw = yaw + ta;
        ++data_->motion_revision;
      } else if (!retreat_clear && data_->retreat_active) {
        // The ReactiveSequence halts BackUp before the fallback replans.
        data_->retreat_active = false;
        data_->retreat_stopped = true;
        ++data_->motion_revision;
      }
    } else if (mode == "escape_available" && escape) {
      data_->turn_handoff = true;
      // An existing successful exit already sends the tree back to planning.
      data_->retreat_active = data_->retreat_stopped = false;
      ++data_->motion_revision;
    }
    cached_mode_ = mode;
    cached_revision_ = data_->motion_revision;
    cached_time_ = now;
    cached_status_ = ((mode == "escape_available" && escape) ||
      (mode == "reverse_needed" && retreat_clear)) ?
      BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    return cached_status_;
  }

  bool fresh(const builtin_interfaces::msg::Time & stamp, double limit) const
  {
    const double age = (node_->now() - rclcpp::Time(stamp)).seconds();
    return age >= -0.1 && age < limit;
  }

  static std::unique_ptr<nav2_costmap_2d::Costmap2D> makeMap(
    const nav2_msgs::msg::Costmap & msg)
  {
    const auto & m = msg.metadata;
    if (m.size_x == 0 || m.size_y == 0 || m.resolution <= 0.0 ||
      msg.data.size() != static_cast<size_t>(m.size_x) * m.size_y)
    {
      throw std::runtime_error("Invalid costmap dimensions");
    }
    auto map = std::make_unique<nav2_costmap_2d::Costmap2D>(
      m.size_x, m.size_y, m.resolution, m.origin.position.x, m.origin.position.y);
    std::copy(msg.data.begin(), msg.data.end(), map->getCharMap());
    return map;
  }

  static bool occupied(
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> & checker,
    double x, double y, double yaw, const nav2_costmap_2d::Footprint & body)
  {
    const double cost = checker.footprintCostAtPose(x, y, yaw, body);
    if (cost < 0.0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
    unsigned int mx, my;
    if (!checker.worldToMap(x, y, mx, my)) {return true;}
    auto * map = checker.getCostmap();
    if (map->getCost(mx, my) >= nav2_costmap_2d::LETHAL_OBSTACLE) {return true;}
    // Nav2's edge checker alone can miss a small obstacle entirely inside
    // the forks/body. Check occupied cell centres inside the polygon too.
    nav2_costmap_2d::Footprint polygon;
    double min_x = std::numeric_limits<double>::infinity(), min_y = min_x;
    double max_x = -min_x, max_y = -min_x;
    for (const auto & p : body) {
      geometry_msgs::msg::Point q;
      q.x = x + std::cos(yaw) * p.x - std::sin(yaw) * p.y;
      q.y = y + std::sin(yaw) * p.x + std::cos(yaw) * p.y;
      polygon.push_back(q);
      min_x = std::min(min_x, q.x); max_x = std::max(max_x, q.x);
      min_y = std::min(min_y, q.y); max_y = std::max(max_y, q.y);
    }
    unsigned int ix0, iy0, ix1, iy1;
    if (!map->worldToMap(min_x, min_y, ix0, iy0) ||
      !map->worldToMap(max_x, max_y, ix1, iy1)) {return true;}
    for (unsigned int ix = ix0; ix <= ix1; ++ix) {
      for (unsigned int iy = iy0; iy <= iy1; ++iy) {
        if (map->getCost(ix, iy) < nav2_costmap_2d::LETHAL_OBSTACLE) {continue;}
        double wx, wy;
        map->mapToWorld(ix, iy, wx, wy);
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
          const auto & a = polygon[i]; const auto & b = polygon[j];
          if ((a.y > wy) != (b.y > wy) &&
            wx < (b.x - a.x) * (wy - a.y) / (b.y - a.y) + a.x)
          {
            inside = !inside;
          }
        }
        if (inside) {return true;}
      }
    }
    return false;
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<ClearanceData> data_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_msgs::msg::Costmap::ConstSharedPtr global_, local_;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr footprint_;
  std::string cached_mode_;
  unsigned cached_revision_{0};
  std::chrono::steady_clock::time_point cached_time_;
  BT::NodeStatus cached_status_{BT::NodeStatus::FAILURE};
};

// Search while stopped, rank complete body-checked routes, revalidate them
// at execution time, and try the next route after a geometric control failure.
// Children: proposal/planning/validation, fresh selected-route check, execution.
class ForwardRouteSearch final : public BT::ControlNode
{
public:
  using BT::ControlNode::ControlNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("candidate_path"),
      BT::InputPort<std::string>("candidate_planner"),
      BT::InputPort<uint16_t>("controller_error", uint16_t{0}, "Execution fault"),
      BT::InputPort<double>("search_timeout", 60.0, "Stopped planning deadline in seconds"),
      BT::OutputPort<double>("candidate_distance"),
      BT::OutputPort<nav_msgs::msg::Path>("path"),
      BT::OutputPort<std::string>("planner_id"),
      BT::OutputPort<double>("local_check_distance")};
  }
  BT::NodeStatus tick() override
  {
    if (childrenCount() != 3) {throw BT::RuntimeError("ForwardRouteSearch needs 3 children");}
    if (deadline_ == std::chrono::steady_clock::time_point{}) {
      const double timeout = getInput<double>("search_timeout").value();
      if (!std::isfinite(timeout) || timeout <= 0.0 || timeout > 120.0) {
        return finish(BT::NodeStatus::FAILURE);
      }
      deadline_ = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(timeout));
    }
    setStatus(BT::NodeStatus::RUNNING);
    if (phase_ == Phase::SEARCH) {
      if (std::chrono::steady_clock::now() >= deadline_) {
        haltChild(0);
        proposal_ = 8;
        RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
          "Forward planning deadline reached; only validated candidates may execute");
      }
      if (proposal_ < 8) {
        setOutput("candidate_distance", 0.15 * (proposal_ + 1));
        const auto result = children_nodes_[0]->executeTick();
        if (result == BT::NodeStatus::RUNNING) {return result;}
        if (result == BT::NodeStatus::SUCCESS) {
          auto path = getInput<nav_msgs::msg::Path>("candidate_path").value();
          std::vector<TrackingPose> poses;
          for (const auto & p : path.poses) {
            poses.push_back({p.pose.position.x, p.pose.position.y,
                tf2::getYaw(p.pose.orientation)});
          }
          const double score = routePreference(poses);
          if (std::isfinite(score)) {
            choices_.push_back({std::move(path),
                getInput<std::string>("candidate_planner").value(), score,
                std::max(1.0, 0.15 * (proposal_ + 1) + 0.05)});
          }
        }
        haltChild(0);
        ++proposal_;
        return BT::NodeStatus::RUNNING;
      }
      std::stable_sort(choices_.begin(), choices_.end(),
        [](const Choice & a, const Choice & b) {return a.score < b.score;});
      RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
        "Forward route search found %zu complete body-checked candidates", choices_.size());
      phase_ = Phase::VALIDATE;
    }
    if (selected_ >= choices_.size()) {return finish(BT::NodeStatus::FAILURE);}
    if (phase_ == Phase::VALIDATE) {
      setOutput("path", choices_[selected_].path);
      setOutput("planner_id", choices_[selected_].planner);
      setOutput("local_check_distance", choices_[selected_].local_range);
      const auto result = children_nodes_[1]->executeTick();
      if (result == BT::NodeStatus::RUNNING) {return result;}
      haltChild(1);
      if (result != BT::NodeStatus::SUCCESS) {
        ++selected_;
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::EXECUTE;
      RCLCPP_INFO(rclcpp::get_logger("forward_route_search"),
        "Trying forward route %zu/%zu, planner %s, length/turn score %.2f",
        selected_ + 1, choices_.size(), choices_[selected_].planner.c_str(),
        choices_[selected_].score);
    }
    const auto result = children_nodes_[2]->executeTick();
    if (result == BT::NodeStatus::RUNNING) {return result;}
    haltChild(2);
    if (result == BT::NodeStatus::SUCCESS) {return finish(result);}
    const auto error = getInput<uint16_t>("controller_error").value();
    if (error != 0 && error != 104 && error != 105 && error != 106) {
      RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
        "Controller fault %u: stopping route retries", error);
      return finish(BT::NodeStatus::FAILURE);
    }
    RCLCPP_WARN(rclcpp::get_logger("forward_route_search"),
      "Forward route execution stopped (controller %u); checking next ranked route", error);
    ++selected_;
    phase_ = Phase::VALIDATE;
    return BT::NodeStatus::RUNNING;
  }
  void halt() override
  {
    BT::ControlNode::halt();
    resetSearch();
  }
private:
  BT::NodeStatus finish(BT::NodeStatus result)
  {
    resetChildren();
    resetSearch();
    return result;
  }
  void resetSearch()
  {
    proposal_ = selected_ = 0;
    phase_ = Phase::SEARCH;
    choices_.clear();
    deadline_ = {};
  }
  struct Choice {
    nav_msgs::msg::Path path;
    std::string planner;
    double score, local_range;
  };
  enum class Phase {SEARCH, VALIDATE, EXECUTE};
  Phase phase_{Phase::SEARCH};
  size_t proposal_{0}, selected_{0};
  std::vector<Choice> choices_;
  std::chrono::steady_clock::time_point deadline_;
};

// Asynchronously prime the shared cache before ANY planner/controller/recovery
// executes. Never turn "no callback received yet" into a geometric collision.
class WaitForClearanceData final : public BT::StatefulActionNode
{
public:
  WaitForClearanceData(const std::string & name, const BT::NodeConfig & config)
  : BT::StatefulActionNode(name, config)
  {
    auto readiness = config;
    readiness.input_ports["data_only"] = "true";
    checker_ = std::make_unique<PathFootprintClear>(name + "_check", readiness);
  }
  static BT::PortsList providedPorts() {return PathFootprintClear::providedPorts();}
  BT::NodeStatus onStart() override
  {
    deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    return onRunning();
  }
  BT::NodeStatus onRunning() override
  {
    if (checker_->executeTick() == BT::NodeStatus::SUCCESS) {return BT::NodeStatus::SUCCESS;}
    return std::chrono::steady_clock::now() < deadline_ ?
           BT::NodeStatus::RUNNING : BT::NodeStatus::FAILURE;
  }
  void onHalted() override {checker_->halt();}

private:
  std::unique_ptr<PathFootprintClear> checker_;
  std::chrono::steady_clock::time_point deadline_;
};

class ResetEscapeState final : public BT::SyncActionNode
{
public:
  using BT::SyncActionNode::SyncActionNode;
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<bool>("reset_motion_history", false, "New goal execution"),
      BT::InputPort<bool>("turn_completed", false, "A checked Spin actually succeeded"),
      BT::InputPort<bool>("invalidate_path", false, "Require planning from the new pose")};
  }
  BT::NodeStatus tick() override
  {
    if (getInput<bool>("turn_completed").value()) {
      std::shared_ptr<ClearanceData> data;
      if (config().blackboard->get("tai_clearance_data", data)) {
        data->failed_forward = data->turn_collision = false;
        if (data->escape_memory.committed && data->selected_turn_goal_facing) {
          data->escape_memory.reset();
        }
        data->selected_turn_goal_facing = data->turn_geometric_stop = false;
        data->turn_handoff = false;
        ++data->motion_revision;
      }
    }
    if (getInput<bool>("reset_motion_history").value()) {
      std::shared_ptr<ClearanceData> data;
      if (config().blackboard->get("tai_clearance_data", data)) {
        data->turn_collision = false;
        data->failed_forward = false;
        data->retreat_active = data->retreat_stopped = false;
        data->forward_active = data->forward_geometric_stop = false;
        data->turn_attempts.clear();
        data->escape_memory.reset();
        data->selected_turn_goal_facing = data->turn_geometric_stop = false;
        data->turn_handoff = false;
        ++data->motion_revision;
      }
    }
    config().blackboard->set<uint16_t>("compute_path_error_code", 0);
    config().blackboard->set<uint16_t>("follow_path_error_code", 0);
    config().blackboard->set<uint16_t>("backup_error_code", 0);
    config().blackboard->set<uint16_t>("escape_forward_error", 0);
    config().blackboard->set<uint16_t>("escape_turn_error", 0);
    config().blackboard->set<bool>("local_escape_required", false);
    config().blackboard->set<std::string>("path_planner", "SE2Fallback");
    if (getInput<bool>("invalidate_path").value()) {
      config().blackboard->set("path", nav_msgs::msg::Path{});
    }
    return BT::NodeStatus::SUCCESS;
  }
};
}  // namespace tai_robot_one

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<tai_robot_one::PathFootprintClear>("PathFootprintClear");
  factory.registerNodeType<tai_robot_one::ForwardRouteSearch>("ForwardRouteSearch");
  factory.registerNodeType<tai_robot_one::ResetEscapeState>("ResetEscapeState");
  factory.registerNodeType<tai_robot_one::WaitForClearanceData>("WaitForClearanceData");
}
