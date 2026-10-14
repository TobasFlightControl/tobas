// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointVelocity.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_path_tools/join.hpp>

#include <tobas_gazebo_msgs/msg/joint_command.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/node_name.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace tobas::gazebo
{
class GazeboJointVelocityControllerPlugin : public BaseNode,
                                            public gz::sim::System,
                                            public gz::sim::ISystemConfigure,
                                            public gz::sim::ISystemPreUpdate
{
  using self = GazeboJointVelocityControllerPlugin;

public:
  explicit GazeboJointVelocityControllerPlugin();

  void Configure(
    const gz::sim::Entity& _model_entity,
    const sdf::ElementConstPtr& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
  gz::sim::Joint joint_;
  const cmp::JointVelocity* jnt_vel_;

  double tar_vel_ = 0.0;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::JointCommand> cmd_sub_;

  void commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& _cmd);
};

GazeboJointVelocityControllerPlugin::GazeboJointVelocityControllerPlugin()
{
}

void GazeboJointVelocityControllerPlugin::Configure(
  const gz::sim::Entity& _model_entity,
  const sdf::ElementConstPtr& _sdf,
  gz::sim::EntityComponentManager& _ecm,
  gz::sim::EventManager&)
{
  const auto joint_name = _sdf->Get<std::string>("jointName");
  initialize("gazebo_" + sanitizeNodeName(joint_name) + "_controller_plugin", _sdf);

  // Get robot model.
  const gz::sim::Model model(_model_entity);
  if (!model.Valid(_ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint.
  const auto joint_entity = model.JointByName(_ecm, joint_name);
  joint_ = gz::sim::Joint(joint_entity);
  if (!joint_.Valid(_ecm)) {
    TOBAS_EXIT("Failed to find joint '", joint_name, "'.");
  }

  // Get joint velocity.
  jnt_vel_ = getComponent<cmp::JointVelocity>(joint_entity, _ecm);

  // Reset joint position.
  const auto home_pos = getSdfParam<double>(_sdf, "homePosition");
  joint_.ResetPosition(_ecm, { home_pos });

  // Register ROS interfaces.
  cmd_sub_ = createSubscriber(path::join(kJointCommandTopicNS, joint_name), &self::commandCb, this);
}

void GazeboJointVelocityControllerPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& _ecm)
{
  joint_.SetVelocity(_ecm, { tar_vel_ });  // This generates torque on the joint.
}

void GazeboJointVelocityControllerPlugin::commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& _cmd)
{
  tar_vel_ = _cmd->data;
}
}  // namespace tobas::gazebo

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointVelocityControllerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
