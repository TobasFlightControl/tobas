// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/moveit/moveit2/blob/HEAD/moveit_core/robot_model/src/revolute_joint_model.cpp
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

#include "tobas_rviz_plugin/joint_model/revolute_joint_model.hpp"

namespace tobas::rviz
{
RevoluteJointModel::RevoluteJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kRevolute;
  variable_names_.push_back(getName());
}

void RevoluteJointModel::getVariableDefaultPositions(double* _values) const
{
  _values[0] = 0.0;
}

void RevoluteJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  const auto c = std::cos(_joint_values[0]);
  const auto s = std::sin(_joint_values[0]);
  const auto t = 1.0 - c;
  const auto txy = t * xy_;
  const auto txz = t * xz_;
  const auto tyz = t * yz_;

  const auto zs = axis_.z() * s;
  const auto ys = axis_.y() * s;
  const auto xs = axis_.x() * s;

  // column major
  auto d = _transform.data();

  d[0] = t * x2_ + c;
  d[1] = txy + zs;
  d[2] = txz - ys;
  d[3] = 0.0;

  d[4] = txy - zs;
  d[5] = t * y2_ + c;
  d[6] = tyz + xs;
  d[7] = 0.0;

  d[8] = txz + ys;
  d[9] = tyz - xs;
  d[10] = t * z2_ + c;
  d[11] = 0.0;

  d[12] = 0.0;
  d[13] = 0.0;
  d[14] = 0.0;
  d[15] = 1.0;
}

void RevoluteJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  Eigen::Quaterniond q(_transform.linear());
  q.normalize();
  size_t max_idx;
  axis_.array().abs().maxCoeff(&max_idx);
  _joint_values[0] = 2.0 * std::atan2(q.vec()[max_idx] / axis_[max_idx], q.w());
}

void RevoluteJointModel::setAxis(const Eigen::Vector3d& _axis)
{
  axis_ = _axis.normalized();
  x2_ = axis_.x() * axis_.x();
  y2_ = axis_.y() * axis_.y();
  z2_ = axis_.z() * axis_.z();
  xy_ = axis_.x() * axis_.y();
  xz_ = axis_.x() * axis_.z();
  yz_ = axis_.y() * axis_.z();
}
}  // namespace tobas::rviz
