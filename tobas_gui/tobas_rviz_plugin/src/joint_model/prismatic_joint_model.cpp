// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/prismatic_joint_model.hpp"

namespace tobas::rviz
{
PrismaticJointModel::PrismaticJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kPrismatic;
  variable_names_.push_back(getName());
}

void PrismaticJointModel::getVariableDefaultPositions(double* _values) const
{
  _values[0] = 0.0;
}

void PrismaticJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  auto d = _transform.data();

  d[0] = 1.0;
  d[1] = 0.0;
  d[2] = 0.0;
  d[3] = 0.0;

  d[4] = 0.0;
  d[5] = 1.0;
  d[6] = 0.0;
  d[7] = 0.0;

  d[8] = 0.0;
  d[9] = 0.0;
  d[10] = 1.0;
  d[11] = 0.0;

  d[12] = axis_.x() * _joint_values[0];
  d[13] = axis_.y() * _joint_values[0];
  d[14] = axis_.z() * _joint_values[0];
  d[15] = 1.0;
}

void PrismaticJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  _joint_values[0] = _transform.translation().dot(axis_);
}

void PrismaticJointModel::setAxis(const Eigen::Vector3d& _axis)
{
  axis_ = _axis;
}
}  // namespace tobas::rviz
