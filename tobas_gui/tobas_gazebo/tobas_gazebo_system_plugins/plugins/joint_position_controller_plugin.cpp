// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <gz/sim/Joint.hh>
#include <gz/sim/Model.hh>

#include <tobas_gazebo_common/constants.hpp>
#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_std_tools/check.hpp>

#include <tobas_gazebo_msgs/msg/joint_command.hpp>

#include "tobas_gazebo_system_plugins/common/common.hpp"
#include "tobas_gazebo_system_plugins/node_name.hpp"

namespace ch = std::chrono;
namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
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
    const gz::sim::Entity& model_entity,
    const sdf::ElementConstPtr& sdf,
    gz::sim::EntityComponentManager& ecm,
    gz::sim::EventManager&) override;

  void PreUpdate(const gz::sim::UpdateInfo& info, gz::sim::EntityComponentManager& ecm) override;

private:
  std::string joint_name_;
  struct Param
  {
    double home_pos;    // [rad]
    double time_const;  // [s]
  } param_;

  std::shared_ptr<gz::sim::Joint> joint_;
  const cmp::JointPosition* jnt_pos_;
  const cmp::JointAxis* jnt_axis_;

  double tar_pos_;

  ros2::SubscriberPtr<tobas_gazebo_msgs::msg::JointCommand> cmd_sub_;

  void getSdfParams(const sdf::ElementConstPtr& sdf);
  void registerRosInterfaces();

  void commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd);
};

GazeboJointPositionControllerPlugin::GazeboJointPositionControllerPlugin()
{
}

void GazeboJointPositionControllerPlugin::Configure(
  const gz::sim::Entity& model_entity,
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  gz::sim::EventManager&)
{
  joint_name_ = sdf->Get<std::string>("jointName");
  initialize("gazebo_" + sanitizeNodeName(joint_name_) + "_controller_plugin", sdf);
  getSdfParams(sdf);

  // Get robot model.
  const gz::sim::Model model(model_entity);
  if (!model.Valid(ecm)) {
    TOBAS_EXIT("Failed to find model.");
  }

  // Get joint.
  const auto joint_entity = model.JointByName(ecm, joint_name_);
  joint_ = std::make_shared<gz::sim::Joint>(joint_entity);
  if (!joint_->Valid(ecm)) {
    TOBAS_EXIT("Failed to find joint \"", joint_name_, "\".");
  }

  // Get joint position.
  TOBAS_CHECK(jnt_pos_ = getComponent<cmp::JointPosition>(joint_entity, ecm));
  TOBAS_CHECK(jnt_axis_ = getComponent<cmp::JointAxis>(joint_entity, ecm));

  // Reset joint position.
  tar_pos_ = param_.home_pos;
  joint_->ResetPosition(ecm, { tar_pos_ });

  // Register ROS interfaces.
  registerRosInterfaces();
}

void GazeboJointPositionControllerPlugin::PreUpdate(const gz::sim::UpdateInfo&, gz::sim::EntityComponentManager& ecm)
{
  const auto& cur_pos = jnt_pos_->Data().at(0);
  const auto tar_vel = (tar_pos_ - cur_pos) / param_.time_const;
  joint_->SetVelocity(ecm, { tar_vel });  // This generates torque on the joint.
}

void GazeboJointPositionControllerPlugin::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  getSdfParam(sdf, "homePosition", param_.home_pos);
  getSdfParam(sdf, "timeConstant", param_.time_const, kPositive);
}

void GazeboJointPositionControllerPlugin::registerRosInterfaces()
{
  cmd_sub_ = createSubscriber(path::join(kJointCommandTopicNS, joint_name_), &self::commandCb, this);
}

void GazeboJointPositionControllerPlugin::commandCb(const tobas_gazebo_msgs::msg::JointCommand::ConstSharedPtr& cmd)
{
  // Clamp the target position with a small margin because commands stop being accepted once the limit is exceeded.
  static constexpr double kJointLimitMargin = 1e-2;  // [rad]
  const auto lower = jnt_axis_->Data().Lower() + kJointLimitMargin;
  const auto upper = jnt_axis_->Data().Upper() - kJointLimitMargin;
  assert(lower < upper);
  tar_pos_ = std::clamp(cmd->data, lower, upper);
}
}  // namespace gazebo
}  // namespace tobas

GZ_ADD_PLUGIN(
  tobas::gazebo::GazeboJointPositionControllerPlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
