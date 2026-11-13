// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/src/planar_joint_model.cpp
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

#include "tobas_rviz_plugin/joint_model/planar_joint_model.hpp"

#include <limits>

namespace tobas::rviz
{
PlanarJointModel::PlanarJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kPlanar;

  for (const auto* variable_name : { "x", "y", "theta" }) {
    variable_names_.push_back(getName() + "/" + variable_name);
  }
}

void PlanarJointModel::getVariableDefaultPositions(double* _values) const
{
  for (size_t i = 0; i < 2; ++i) {
    _values[i] = 0.0;
  }
  _values[2] = 0.0;
}

void PlanarJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  _transform = Eigen::Isometry3d(
    Eigen::Translation3d(_joint_values[0], _joint_values[1], 0.0) *
    Eigen::AngleAxisd(_joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  _joint_values[0] = _transform.translation().x();
  _joint_values[1] = _transform.translation().y();

  const Eigen::Quaterniond q(_transform.linear());

  // Taken from Bullet
  const auto s_squared = 1.0 - (q.w() * q.w());
  if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
    _joint_values[2] = 0.0;
  }
  else {
    const auto s = 1.0 / std::sqrt(s_squared);
    _joint_values[2] = (std::acos(q.w()) * 2.0f) * (q.z() * s);
  }
}
}  // namespace tobas::rviz
