// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/conversions.hpp"

#include <QDebug>
#include <tf2_eigen/tf2_eigen.hpp>

namespace tobas::rviz
{
namespace
{
bool jointStateToRobotState(const sensor_msgs::msg::JointState& _joint_state, RobotState& _state)
{
  if (_joint_state.name.size() != _joint_state.position.size()) {
    qCritical().nospace() << "Different number of names and positions in JointState message: "
                          << _joint_state.name.size() << ", " << _joint_state.position.size();
    return false;
  }

  _state.setVariablePositions(_joint_state.name, _joint_state.position);
  return true;
}

bool multiDofJointsToRobotState(const sensor_msgs::msg::MultiDOFJointState& _multi_dof_state, RobotState& _state)
{
  if (_multi_dof_state.joint_names.size() != _multi_dof_state.transforms.size()) {
    qCritical() << "Different number of names and transforms in MultiDOFJointState message.";
    return false;
  }

  bool valid = true;
  if (!_multi_dof_state.joint_names.empty() && _multi_dof_state.header.frame_id != _state.getRobotModel().getModelFrame()) {
    qWarning().nospace() << "The transform for multi-dof joints was specified in frame '"
                         << _multi_dof_state.header.frame_id.c_str()
                         << "' but it was not possible to transform that to frame '"
                         << _state.getRobotModel().getModelFrame().c_str() << "'.";
    valid = false;
  }

  for (size_t i = 0; i < _multi_dof_state.joint_names.size(); ++i) {
    const auto& joint_name = _multi_dof_state.joint_names[i];
    if (!_state.getRobotModel().hasJointModel(joint_name)) {
      qWarning().nospace() << "No joint matching multi-dof joint '" << joint_name.c_str() << "'.";
      valid = false;
      continue;
    }
    _state.setJointPositions(joint_name, tf2::transformToEigen(_multi_dof_state.transforms[i]));
  }

  return valid;
}
}  // namespace

bool robotStateMsgToRobotState(const tobas_visualization_msgs::msg::RobotState& _robot_state, RobotState& _state)
{
  if (
    !_robot_state.is_diff && _robot_state.joint_state.name.empty() &&
    _robot_state.multi_dof_joint_state.joint_names.empty()) {
    qCritical() << "Found empty JointState message.";
    return false;
  }

  const bool joint_state_valid = jointStateToRobotState(_robot_state.joint_state, _state);
  const bool multi_dof_state_valid = multiDofJointsToRobotState(_robot_state.multi_dof_joint_state, _state);
  _state.update();
  return joint_state_valid || multi_dof_state_valid;
}
}  // namespace tobas::rviz
