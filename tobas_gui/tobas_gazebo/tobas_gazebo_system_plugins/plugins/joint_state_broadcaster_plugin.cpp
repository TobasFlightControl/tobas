// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <chrono>
#include <optional>

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>

#include <tobas_msgs/msg/joint_state_array.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/rate_manager.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
class GazeboJointStateBroadcasterPlugin : public BaseNode,
                                          public gz::sim::System,
                                          public gz::sim::ISystemConfigure,
                                          public gz::sim::ISystemPostUpdate
{
public:
  explicit GazeboJointStateBroadcasterPlugin();

  void Configure(
    const gz::sim::Entity& _model_entity,
    const sdf::ElementConstPtr& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& _info, const gz::sim::EntityComponentManager& _ecm) override;

private:
  std::vector<std::string> joint_names_;

  std::map<std::string, const cmp::JointPosition*> jnt_pos_;
  std::map<std::string, const cmp::JointVelocity*> jnt_vel_;
  std::map<std::string, const cmp::JointTransmittedWrench*> jnt_eff_;

  std::optional<RateManager> rate_manager_;

  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> js_pub_;
};

GazeboJointStateBroadcasterPlugin::GazeboJointStateBroadcasterPlugin()
{
}

void GazeboJointStateBroadcasterPlugin::Configure(
  const gz::sim::Entity& _model_entity,
  const sdf::ElementConstPtr& _sdf,
  gz::sim::EntityComponentManager& _ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_joint_state_broadcaster_plugin", _sdf);

  // Get robot model.
  const gz::sim::Model model(_model_entity);
  if (!model.Valid(_ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint states.
  joint_names_ = getSdfParam<std::vector<std::string>>(_sdf, "jointNames");
  for (const auto& jnt_name : joint_names_) {
    const auto joint_entity = model.JointByName(_ecm, jnt_name);
    const auto joint = std::make_shared<gz::sim::Joint>(joint_entity);
    if (!joint->Valid(_ecm)) {
      TOBAS_EXIT("Failed to find joint '", jnt_name, "'.");
    }
    jnt_pos_[jnt_name] = getComponent<cmp::JointPosition>(joint_entity, _ecm);
    jnt_vel_[jnt_name] = getComponent<cmp::JointVelocity>(joint_entity, _ecm);
    jnt_eff_[jnt_name] = getComponent<cmp::JointTransmittedWrench>(joint_entity, _ecm);
  }

  // Set update rate.
  const auto update_rate = getSdfParam<int>(_sdf, "updateRate", kNonNegative);
  rate_manager_.emplace(update_rate);

  // Register ROS interfaces.
  js_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(topic::kJointStates);
}

void GazeboJointStateBroadcasterPlugin::PostUpdate(
  const gz::sim::UpdateInfo& _info,
  const gz::sim::EntityComponentManager&)
{
  if (_info.paused || _info.dt <= ch::steady_clock::duration::zero()) {
    return;
  }

  if (!rate_manager_->update(_info.simTime)) {
    return;
  }

  auto js = std::make_unique<tobas_msgs::msg::JointStateArray>();
  ros2::timeChronoToMsg(_info.simTime, js->header.stamp);

  for (const auto& jnt_name : joint_names_) {
    tobas_msgs::msg::JointState state;
    state.name = jnt_name;
    state.position = jnt_pos_.at(jnt_name)->Data().at(0);
    state.velocity = jnt_vel_.at(jnt_name)->Data().at(0);
    state.effort = gz::msgs::Convert(jnt_eff_.at(jnt_name)->Data().torque()).Length();
    js->states.push_back(state);
  }

  js_pub_->publish(std::move(js));
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointStateBroadcasterPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
