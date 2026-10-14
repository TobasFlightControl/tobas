// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/src/floating_joint_model.cpp
// Modified by Tobas, Inc.

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2008-2013, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include "tobas_rviz_plugin/joint_model/floating_joint_model.hpp"

#include <eigen3/Eigen/Geometry>

namespace tobas::rviz
{
FloatingJointModel::FloatingJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kFloating;
  for (const auto* variable_name : { "trans_x", "trans_y", "trans_z", "rot_x", "rot_y", "rot_z", "rot_w" }) {
    variable_names_.push_back(getName() + "/" + variable_name);
  }
}

void FloatingJointModel::getVariableDefaultPositions(double* _values) const
{
  for (size_t i = 0; i < 3; ++i) {
    _values[i] = 0.0;
  }

  _values[3] = 0.0;
  _values[4] = 0.0;
  _values[5] = 0.0;
  _values[6] = 1.0;
}

void FloatingJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  _transform = Eigen::Isometry3d(
    Eigen::Translation3d(_joint_values[0], _joint_values[1], _joint_values[2]) *
    Eigen::Quaterniond(_joint_values[6], _joint_values[3], _joint_values[4], _joint_values[5]).normalized());
}

void FloatingJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  _joint_values[0] = _transform.translation().x();
  _joint_values[1] = _transform.translation().y();
  _joint_values[2] = _transform.translation().z();
  const Eigen::Quaterniond q(_transform.linear());
  _joint_values[3] = q.x();
  _joint_values[4] = q.y();
  _joint_values[5] = q.z();
  _joint_values[6] = q.w();
}
}  // namespace tobas::rviz
