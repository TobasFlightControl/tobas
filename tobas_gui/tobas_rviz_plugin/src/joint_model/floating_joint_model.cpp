// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/floating_joint_model.hpp"

#include <geometric_shapes/check_isometry.h>
#include <eigen3/Eigen/Geometry>

namespace tobas
{
FloatingJointModel::FloatingJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kFloating;
  local_variable_names_.push_back("trans_x");
  local_variable_names_.push_back("trans_y");
  local_variable_names_.push_back("trans_z");
  local_variable_names_.push_back("rot_x");
  local_variable_names_.push_back("rot_y");
  local_variable_names_.push_back("rot_z");
  local_variable_names_.push_back("rot_w");
  for (size_t i = 0; i < 7; ++i) {
    variable_names_.push_back(getName() + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }
}

void FloatingJointModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < 3; ++i) {
    values[i] = 0.;
  }

  values[3] = 0.;
  values[4] = 0.;
  values[5] = 0.;
  values[6] = 1.;
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
  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain non-isometry
  const Eigen::Quaterniond q(transform.linear());
  joint_values[3] = q.x();
  joint_values[4] = q.y();
  joint_values[5] = q.z();
  joint_values[6] = q.w();
}
}  // namespace tobas
