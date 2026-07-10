// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/conversions.hpp"

#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.conversions");
}

bool jointStateToRobotState(const sensor_msgs::msg::JointState& joint_state, RobotState& state)
{
  if (joint_state.name.size() != joint_state.position.size()) {
    RCLCPP_ERROR(
      getLogger(),
      "Different number of names and positions in JointState message: %zu, %zu",
      joint_state.name.size(),
      joint_state.position.size());
    return false;
  }

  state.setVariablePositions(joint_state.name, joint_state.position);
  return true;
}

bool multiDofJointsToRobotState(const sensor_msgs::msg::MultiDOFJointState& multi_dof_state, RobotState& state)
{
  if (multi_dof_state.joint_names.size() != multi_dof_state.transforms.size()) {
    RCLCPP_ERROR(getLogger(), "Different number of names and transforms in MultiDOFJointState message.");
    return false;
  }

  bool valid = true;
  if (!multi_dof_state.joint_names.empty() && multi_dof_state.header.frame_id != state.getRobotModel()->getModelFrame()) {
    RCLCPP_WARN(
      getLogger(),
      "The transform for multi-dof joints was specified in frame '%s' "
      "but it was not possible to transform that to frame '%s'.",
      multi_dof_state.header.frame_id.c_str(),
      state.getRobotModel()->getModelFrame().c_str());
    valid = false;
  }

  for (size_t i = 0; i < multi_dof_state.joint_names.size(); ++i) {
    const auto& joint_name = multi_dof_state.joint_names[i];
    if (!state.getRobotModel()->hasJointModel(joint_name)) {
      RCLCPP_WARN(getLogger(), "No joint matching multi-dof joint '%s'.", joint_name.c_str());
      valid = false;
      continue;
    }
    state.setJointPositions(joint_name, tf2::transformToEigen(multi_dof_state.transforms[i]));
  }

  return valid;
}
}  // namespace

bool robotStateMsgToRobotState(const tobas_visualization_msgs::msg::RobotState& robot_state, RobotState& state)
{
  if (!robot_state.is_diff && robot_state.joint_state.name.empty() && robot_state.multi_dof_joint_state.joint_names.empty()) {
    RCLCPP_ERROR(getLogger(), "Found empty JointState message.");
    return false;
  }

  const bool joint_state_valid = jointStateToRobotState(robot_state.joint_state, state);
  const bool multi_dof_state_valid = multiDofJointsToRobotState(robot_state.multi_dof_joint_state, state);
  state.update();
  return joint_state_valid || multi_dof_state_valid;
}
}  // namespace tobas
