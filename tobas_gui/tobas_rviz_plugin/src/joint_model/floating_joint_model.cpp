// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/floating_joint_model.hpp"

#include <eigen3/Eigen/Geometry>

namespace tobas
{
FloatingJointModel::FloatingJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kFloating;
  for (const auto* variable_name : { "trans_x", "trans_y", "trans_z", "rot_x", "rot_y", "rot_z", "rot_w" }) {
    variable_names_.push_back(getName() + "/" + variable_name);
  }
}

void FloatingJointModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < 3; ++i) {
    values[i] = 0.0;
  }

  values[3] = 0.0;
  values[4] = 0.0;
  values[5] = 0.0;
  values[6] = 1.0;
}

void FloatingJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  transform = Eigen::Isometry3d(
    Eigen::Translation3d(joint_values[0], joint_values[1], joint_values[2]) *
    Eigen::Quaterniond(joint_values[6], joint_values[3], joint_values[4], joint_values[5]).normalized());
}

void FloatingJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().x();
  joint_values[1] = transform.translation().y();
  joint_values[2] = transform.translation().z();
  const Eigen::Quaterniond q(transform.linear());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}
}  // namespace tobas
