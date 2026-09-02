// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboGroundTruthStatePlugin : public BaseNode,
                                     public gz::sim::System,
                                     public gz::sim::ISystemConfigure,
                                     public gz::sim::ISystemPostUpdate
{
  using self = GazeboGroundTruthStatePlugin;

public:
  explicit GazeboGroundTruthStatePlugin();

  void Configure(
    const gz::sim::Entity& model,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  // SDF parameters
  std::string link_name_;
  int update_rate_;

  const cmp::WorldPose* pose_W_;
  const cmp::LinearVelocity* vel_B_;
  const cmp::AngularVelocity* gyro_B_;
  const cmp::LinearAcceleration* acc_B_;
  const cmp::AngularAcceleration* dgyro_B_;

  std::optional<RateManager> rate_manager_;

  ros2::PublisherPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
};

GazeboGroundTruthStatePlugin::GazeboGroundTruthStatePlugin()
{
}

void GazeboGroundTruthStatePlugin::Configure(
  const gz::sim::Entity& model,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_ground_truth_state_plugin", sdf);
  getSdfParams(sdf);

  const auto link = ecm.EntityByComponents(cmp::Link(), cmp::ParentEntity(model), cmp::Name(link_name_));
  if (link == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link \"", link_name_, "\".");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link, ecm);
  vel_B_ = getComponent<cmp::LinearVelocity>(link, ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link, ecm);
  acc_B_ = getComponent<cmp::LinearAcceleration>(link, ecm);
  dgyro_B_ = getComponent<cmp::AngularAcceleration>(link, ecm);

  rate_manager_.emplace(update_rate_);

  odom_pub_ = createPublisher<tobas_msgs::OdometryWithCovarianceStamped>(kOdometryGtTopic);
}

void GazeboGroundTruthStatePlugin::PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  // Create Pose & Twist message.
  auto odom = std::make_unique<tobas_msgs::OdometryWithCovarianceStamped>();
  odom->header.frame_id = link_name_;

  // Update time stamp.
  ros2::timeChronoToMsg(info.simTime, odom->header.stamp);

  // Update pose (Global).
  poseGazeboToKDL(pose_W_->Data(), odom->odom.odom.frame);

  // Update linear velocity (Local).
  vectorGazeboToKDL(vel_B_->Data(), odom->odom.odom.twist.vel);

  // Update angular velocity (Local).
  vectorGazeboToKDL(gyro_B_->Data(), odom->odom.odom.twist.rot);

  // Update linear acceleration (Local).
  vectorGazeboToKDL(acc_B_->Data(), odom->odom.odom.accel.linear);

  // Update angular acceleration (Local).
  vectorGazeboToKDL(dgyro_B_->Data(), odom->odom.odom.accel.angular);

  // Update covariances.
  odom->odom.position_covariance.setZero();
  odom->odom.orientation_covariance.setZero();
  odom->odom.velocity_covariance.setZero();
  odom->odom.gyro_covariance.setZero();

  // Publish state message.
  odom_pub_->publish(std::move(odom));
}

void GazeboGroundTruthStatePlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  constexpr int kDefaultUpdateRate = 0;  // [Hz]

  getSdfParam(sdf, "linkName", link_name_);
  getSdfParam(sdf, "updateRate", update_rate_, kDefaultUpdateRate, kNonNegative);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboGroundTruthStatePlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
