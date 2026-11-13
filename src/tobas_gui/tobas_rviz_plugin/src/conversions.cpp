// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_state/src/conversions.cpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2011-2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Dave Coleman */

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
