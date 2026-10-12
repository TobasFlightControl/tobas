// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/floating_joint_model.hpp"

#include <eigen3/Eigen/Geometry>

namespace tobas
{
namespace rviz
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
}  // namespace rviz
}  // namespace tobas
