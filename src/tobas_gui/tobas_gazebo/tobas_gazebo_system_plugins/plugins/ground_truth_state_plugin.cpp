// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <optional>

#include <gz/sim/Model.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_conversions/gazebo_kdl.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

namespace cmp = gz::sim::components;

namespace tobas::gazebo
{
class GazeboGroundTruthStatePlugin : public BaseNode,
                                     public gz::sim::System,
                                     public gz::sim::ISystemConfigure,
                                     public gz::sim::ISystemPostUpdate
{
public:
  explicit GazeboGroundTruthStatePlugin();

  void Configure(
    const gz::sim::Entity& _model_entity,
    const sdf::ElementConstPtr& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) override;

private:
  const cmp::WorldPose* pose_W_;
  const cmp::LinearVelocity* vel_B_;
  const cmp::AngularVelocity* gyro_B_;
  const cmp::LinearAcceleration* acc_B_;
  const cmp::AngularAcceleration* dgyro_B_;

  std::optional<RateManager> rate_manager_;

  ros2::PublisherPtr<tobas_msgs::OdometryWithCovarianceStamped> odom_pub_;
};

GazeboGroundTruthStatePlugin::GazeboGroundTruthStatePlugin()
{
}

void GazeboGroundTruthStatePlugin::Configure(
  const gz::sim::Entity& _model_entity,
  const sdf::ElementConstPtr& _sdf,
  gz::sim::EntityComponentManager& _ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_ground_truth_state_plugin", _sdf);

  const gz::sim::Model model(_model_entity);
  const auto link_name = getSdfParam<std::string>(_sdf, "linkName");
  const auto link_enitty = model.LinkByName(_ecm, link_name);
  if (link_enitty == gz::sim::kNullEntity) {
    TOBAS_EXIT("Failed to find specified link '", link_name, "'.");
  }

  pose_W_ = getComponent<cmp::WorldPose>(link_enitty, _ecm);
  vel_B_ = getComponent<cmp::LinearVelocity>(link_enitty, _ecm);
  gyro_B_ = getComponent<cmp::AngularVelocity>(link_enitty, _ecm);
  acc_B_ = getComponent<cmp::LinearAcceleration>(link_enitty, _ecm);
  dgyro_B_ = getComponent<cmp::AngularAcceleration>(link_enitty, _ecm);

  const auto update_rate = getSdfParam<int>(_sdf, "updateRate", 0, kNonNegative);
  rate_manager_.emplace(update_rate);

  odom_pub_ = createPublisher<tobas_msgs::OdometryWithCovarianceStamped>(kOdometryGtTopic);
}

void GazeboGroundTruthStatePlugin::PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager&)
{
  if (_info.paused || _info.dt <= std::chrono::steady_clock::duration::zero()) {
    return;
  }

  if (!rate_manager_->update(_info.simTime)) {
    return;
  }

  // Create Pose & Twist message.
  auto odom = std::make_unique<tobas_msgs::OdometryWithCovarianceStamped>();

  // Update time stamp.
  ros2::timeChronoToMsg(_info.simTime, odom->header.stamp);

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
}  // namespace tobas::gazebo

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboGroundTruthStatePlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
