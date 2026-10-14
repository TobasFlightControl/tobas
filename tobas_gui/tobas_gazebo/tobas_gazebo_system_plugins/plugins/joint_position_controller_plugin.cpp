// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointPosition.hh>

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
class GazeboJointPositionControllerPlugin : public BaseNode,
                                            public gz::sim::System,
                                            public gz::sim::ISystemConfigure,
                                            public gz::sim::ISystemPreUpdate
{
  using self = GazeboJointPositionControllerPlugin;

public:
  explicit GazeboJointPositionControllerPlugin();

  void Configure(
    const gz::sim::Entity& _model_entity,
    const sdf::ElementConstPtr& _sdf,
    gz::sim::EntityComponentManager& _ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) override;

private:
  double time_const_;  ///< [s]
  double tar_pos_;

  gz::sim::Joint joint_;
  const cmp::JointPosition* jnt_pos_;
  const cmp::JointAxis* jnt_axis_;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::JointCommand> cmd_sub_;

  void commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& _cmd);
};

GazeboJointPositionControllerPlugin::GazeboJointPositionControllerPlugin()
{
}

void GazeboJointPositionControllerPlugin::Configure(
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

  // Get joint position.
  jnt_pos_ = getComponent<cmp::JointPosition>(joint_entity, _ecm);
  jnt_axis_ = getComponent<cmp::JointAxis>(joint_entity, _ecm);

  // Reset joint position.
  tar_pos_ = getSdfParam<double>(_sdf, "homePosition");
  joint_.ResetPosition(_ecm, { tar_pos_ });

  // Configure the controller.
  time_const_ = getSdfParam<double>(_sdf, "timeConstant", kPositive);

  // Register ROS interfaces.
  cmd_sub_ = createSubscriber(path::join(kJointCommandTopicNS, joint_name), &self::commandCb, this);
}

void GazeboJointPositionControllerPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& _ecm)
{
  const auto& cur_pos = jnt_pos_->Data().at(0);
  const auto tar_vel = (tar_pos_ - cur_pos) / time_const_;
  joint_.SetVelocity(_ecm, { tar_vel });  // This generates torque on the joint.
}

void GazeboJointPositionControllerPlugin::commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& _cmd)
{
  // Clamp the target position with a small margin because commands stop being accepted once the limit is exceeded.
  constexpr double kJointLimitMargin = 1e-2;  // [rad]
  const auto lower = jnt_axis_->Data().Lower() + kJointLimitMargin;
  const auto upper = jnt_axis_->Data().Upper() - kJointLimitMargin;
  assert(lower < upper);
  tar_pos_ = std::clamp(_cmd->data, lower, upper);
}
}  // namespace tobas::gazebo

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointPositionControllerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
