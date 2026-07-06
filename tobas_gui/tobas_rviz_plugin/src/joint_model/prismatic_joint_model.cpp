// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/prismatic_joint_model.hpp"

namespace tobas
{
PrismaticJointModel::PrismaticJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kPrismatic;
  variable_names_.push_back(getName());
  variable_index_map_[getName()] = 0;
}

void PrismaticJointModel::getVariableDefaultPositions(double* values) const
{
  values[0] = 0.0;
}

void PrismaticJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  auto d = transform.data();

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

  d[12] = axis_.x() * joint_values[0];
  d[13] = axis_.y() * joint_values[0];
  d[14] = axis_.z() * joint_values[0];
  d[15] = 1.0;
}

void PrismaticJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().dot(axis_);
}

/* Get the axis of translation. */
const Eigen::Vector3d& PrismaticJointModel::getAxis() const
{
  return axis_;
}

/* Set the axis of translation. */
void PrismaticJointModel::setAxis(const Eigen::Vector3d& axis)
{
  axis_ = axis;
}
}  // namespace tobas
