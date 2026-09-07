// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <optional>

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_ros2_tools/time.hpp>
#include <tobas_std_tools/check.hpp>

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
  using self = GazeboJointStateBroadcasterPlugin;

public:
  explicit GazeboJointStateBroadcasterPlugin();

  void Configure(
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PostUpdate(const gz::sim::UpdateInfo& info, const gz::sim::EntityComponentManager& ecm) override;

private:
  struct Param
  {
    std::vector<std::string> joint_names;
    int update_rate;
  } param_;

  std::map<std::string, const cmp::JointPosition*> jnt_pos_;
  std::map<std::string, const cmp::JointVelocity*> jnt_vel_;
  std::map<std::string, const cmp::JointTransmittedWrench*> jnt_eff_;

  std::optional<RateManager> rate_manager_;

  ros2::PublisherPtr<tobas_msgs::msg::JointStateArray> js_pub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerRosInterfaces();
};

GazeboJointStateBroadcasterPlugin::GazeboJointStateBroadcasterPlugin()
{
}

void GazeboJointStateBroadcasterPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  initialize("gazebo_joint_state_broadcaster_plugin", sdf);
  getSdfParams(sdf);

  rate_manager_.emplace(param_.update_rate);

  // Get robot model.
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint states.
  for (const auto& jnt_name : param_.joint_names) {
    const auto joint_entity = model.JointByName(ecm, jnt_name);
    const auto joint = std::make_shared<gz::sim::Joint>(joint_entity);
    if (!joint->Valid(ecm)) {
      TOBAS_EXIT("Failed to find joint '", jnt_name, "'.");
    }
    TOBAS_CHECK(jnt_pos_[jnt_name] = getComponent<cmp::JointPosition>(joint_entity, ecm));
    TOBAS_CHECK(jnt_vel_[jnt_name] = getComponent<cmp::JointVelocity>(joint_entity, ecm));
    TOBAS_CHECK(jnt_eff_[jnt_name] = getComponent<cmp::JointTransmittedWrench>(joint_entity, ecm));
  }

  // Register ROS interfaces.
  registerRosInterfaces();
}

void GazeboJointStateBroadcasterPlugin::PostUpdate(
  const gz::sim::UpdateInfo& info,
  const gz::sim::EntityComponentManager&)
{
  if (!rate_manager_->update(info.simTime)) {
    return;
  }

  auto js = std::make_unique<tobas_msgs::msg::JointStateArray>();
  ros2::timeChronoToMsg(info.simTime, js->header.stamp);

  for (const auto& jnt_name : param_.joint_names) {
    tobas_msgs::msg::JointState state;
    state.name = jnt_name;
    state.position = jnt_pos_.at(jnt_name)->Data().at(0);
    state.velocity = jnt_vel_.at(jnt_name)->Data().at(0);
    state.effort = gz::msgs::Convert(jnt_eff_.at(jnt_name)->Data().torque()).Length();
    js->states.push_back(state);
  }

  js_pub_->publish(std::move(js));
}

void GazeboJointStateBroadcasterPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "jointNames", param_.joint_names);
  getSdfParam(sdf, "updateRate", param_.update_rate, kNonNegative);
}

void GazeboJointStateBroadcasterPlugin::registerRosInterfaces()
{
  js_pub_ = createPublisher<tobas_msgs::msg::JointStateArray>(topic::kJointStates);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointStateBroadcasterPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPostUpdate)
