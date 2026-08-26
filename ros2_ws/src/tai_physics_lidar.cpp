// Copyright 2026 TAI
// SPDX-License-Identifier: Apache-2.0

#include <gz/msgs/laserscan.pb.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <random>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/transport/Node.hh>

namespace tai_robot_one
{

constexpr double kPi = 3.14159265358979323846;

class PhysicsLidar final :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity & model_entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager &) override
  {
    this->frame_id_ = sdf->Get<std::string>("frame_id", "lidar_link").first;
    this->parent_link_ =
      sdf->Get<std::string>("parent_link", "base_footprint").first;
    this->sensor_pose_ = sdf->Get<gz::math::Pose3d>(
      "sensor_pose", gz::math::Pose3d::Zero).first;
    this->topic_ = sdf->Get<std::string>("topic", "/scan").first;
    this->samples_ = sdf->Get<unsigned int>("samples", 1143u).first;
    this->update_rate_ = sdf->Get<double>("update_rate", 7.0).first;
    this->angle_min_ = sdf->Get<double>("angle_min", -kPi).first;
    this->angle_max_ = sdf->Get<double>("angle_max", kPi).first;
    this->range_min_ = sdf->Get<double>("range_min", 0.15).first;
    this->range_max_ = sdf->Get<double>("range_max", 12.0).first;
    this->noise_stddev_ = sdf->Get<double>("noise_stddev", 0.003).first;

    if (this->samples_ < 2u || this->update_rate_ <= 0.0 ||
      this->angle_max_ <= this->angle_min_ ||
      this->range_min_ < 0.0 || this->range_max_ <= this->range_min_)
    {
      gzerr << "PhysicsLidar received invalid scan parameters" << std::endl;
      return;
    }

    this->lidar_entity_ =
      gz::sim::Model(model_entity).LinkByName(ecm, this->parent_link_);
    if (this->lidar_entity_ == gz::sim::kNullEntity)
    {
      gzerr << "PhysicsLidar could not find physics parent link ["
            << this->parent_link_ << "]" << std::endl;
      return;
    }
    if (ecm.Component<gz::sim::components::Pose>(this->lidar_entity_) == nullptr)
    {
      gzerr << "PhysicsLidar frame [" << this->frame_id_
            << "] has no Pose component" << std::endl;
      this->lidar_entity_ = gz::sim::kNullEntity;
      return;
    }

    this->raycast_request_.rays.reserve(this->samples_);
    this->angle_step_ =
      (this->angle_max_ - this->angle_min_) /
      static_cast<double>(this->samples_ - 1u);

    for (unsigned int index = 0; index < this->samples_; ++index)
    {
      const double angle =
        this->angle_min_ + static_cast<double>(index) * this->angle_step_;
      const gz::math::Vector3d local_direction{
        std::cos(angle), std::sin(angle), 0.0};
      const gz::math::Vector3d direction =
        this->sensor_pose_.Rot().RotateVector(local_direction);
      this->raycast_request_.rays.push_back({
        this->sensor_pose_.Pos() + direction * this->range_min_,
        this->sensor_pose_.Pos() + direction * this->range_max_});
    }

    this->publisher_ =
      this->transport_node_.Advertise<gz::msgs::LaserScan>(this->topic_);
    if (!this->publisher_)
    {
      gzerr << "PhysicsLidar failed to advertise [" << this->topic_ << "]"
            << std::endl;
      this->lidar_entity_ = gz::sim::kNullEntity;
      return;
    }

    this->update_period_ = std::chrono::duration_cast<
      std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->update_rate_));
    this->configured_ = true;
    gzmsg << "PhysicsLidar publishing " << this->samples_ << " physics rays at "
          << this->update_rate_ << " Hz on [" << this->topic_
          << "] in frame [" << this->frame_id_ << "]" << std::endl;
  }

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured_ || info.paused ||
      this->lidar_entity_ == gz::sim::kNullEntity)
    {
      return;
    }

    if (info.simTime < this->last_sim_time_)
    {
      ecm.RemoveComponent<gz::sim::components::RaycastData>(
        this->lidar_entity_);
      this->raycast_active_ = false;
      this->remove_raycast_ = false;
      this->next_update_time_ = info.simTime;
    }
    this->last_sim_time_ = info.simTime;

    // RaycastData is intentionally present for only one physics cycle per
    // scan. Keeping it attached permanently would cast 1143 rays at the world
    // physics rate (200 Hz), even though this LiDAR publishes only at 7 Hz.
    if (this->remove_raycast_)
    {
      ecm.RemoveComponent<gz::sim::components::RaycastData>(
        this->lidar_entity_);
      this->remove_raycast_ = false;
    }

    // RaycastData results become available after the physics update. Start
    // the request one physics step before the nominal scan deadline so the
    // published /scan rate remains 7 Hz instead of losing one 5 ms step.
    if (!this->raycast_active_ &&
      info.simTime + info.dt >= this->next_update_time_)
    {
      ecm.CreateComponent(
        this->lidar_entity_,
        gz::sim::components::RaycastData(this->raycast_request_));
      this->raycast_active_ = true;
    }
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & info,
    const gz::sim::EntityComponentManager & ecm) override
  {
    if (!this->configured_ || info.paused ||
      this->lidar_entity_ == gz::sim::kNullEntity)
    {
      return;
    }

    if (!this->raycast_active_)
    {
      return;
    }

    const auto * raycast =
      ecm.Component<gz::sim::components::RaycastData>(this->lidar_entity_);
    if (raycast == nullptr || raycast->Data().results.size() != this->samples_)
    {
      return;
    }

    gz::msgs::LaserScan scan;
    const auto seconds =
      std::chrono::duration_cast<std::chrono::seconds>(info.simTime);
    const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
      info.simTime - seconds);
    scan.mutable_header()->mutable_stamp()->set_sec(seconds.count());
    scan.mutable_header()->mutable_stamp()->set_nsec(
      static_cast<int32_t>(nanoseconds.count()));
    scan.set_frame(this->frame_id_);
    scan.set_angle_min(this->angle_min_);
    scan.set_angle_max(this->angle_max_);
    scan.set_angle_step(this->angle_step_);
    scan.set_range_min(this->range_min_);
    scan.set_range_max(this->range_max_);
    scan.set_count(this->samples_);
    scan.set_vertical_angle_min(0.0);
    scan.set_vertical_angle_max(0.0);
    scan.set_vertical_angle_step(0.0);
    scan.set_vertical_count(1u);

    std::normal_distribution<double> noise(0.0, this->noise_stddev_);
    for (const auto & result : raycast->Data().results)
    {
      double range = std::numeric_limits<double>::infinity();
      if (std::isfinite(result.fraction) && result.fraction >= 0.0 &&
        result.fraction < 1.0)
      {
        range = (result.point - this->sensor_pose_.Pos()).Length();
        if (this->noise_stddev_ > 0.0)
        {
          range += noise(this->random_engine_);
        }
        range = std::clamp(range, this->range_min_, this->range_max_);
      }
      scan.add_ranges(range);
      scan.add_intensities(std::isfinite(range) ? 1.0 : 0.0);
    }

    this->publisher_.Publish(scan);
    this->raycast_active_ = false;
    this->remove_raycast_ = true;
    do
    {
      this->next_update_time_ += this->update_period_;
    } while (this->next_update_time_ <= info.simTime);
  }

private:
  gz::sim::Entity lidar_entity_{gz::sim::kNullEntity};
  gz::sim::components::RaycastDataInfo raycast_request_;
  gz::transport::Node transport_node_;
  gz::transport::Node::Publisher publisher_;
  std::string frame_id_{"lidar_link"};
  std::string parent_link_{"base_footprint"};
  std::string topic_{"/scan"};
  gz::math::Pose3d sensor_pose_{gz::math::Pose3d::Zero};
  unsigned int samples_{1143u};
  double update_rate_{7.0};
  double angle_min_{-kPi};
  double angle_max_{kPi};
  double angle_step_{0.0};
  double range_min_{0.15};
  double range_max_{12.0};
  double noise_stddev_{0.003};
  std::chrono::steady_clock::duration update_period_{};
  std::chrono::steady_clock::duration next_update_time_{};
  std::chrono::steady_clock::duration last_sim_time_{};
  std::mt19937 random_engine_{0x544149u};
  bool raycast_active_{false};
  bool remove_raycast_{false};
  bool configured_{false};
};

}  // namespace tai_robot_one

GZ_ADD_PLUGIN(
  tai_robot_one::PhysicsLidar,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(
  tai_robot_one::PhysicsLidar,
  "tai_robot_one::PhysicsLidar")
