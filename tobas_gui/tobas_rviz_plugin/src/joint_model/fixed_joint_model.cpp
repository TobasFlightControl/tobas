// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/fixed_joint_model.hpp"

namespace tobas
{
FixedJointModel::FixedJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = FIXED;
}

uint32_t FixedJointModel::getStateSpaceDimension() const
{
  return 0;
}

void FixedJointModel::getVariableDefaultPositions(double* /*values*/, const Bounds& /*bounds*/) const
{
}

void FixedJointModel::computeTransform(const double* /* joint_values */, Eigen::Isometry3d& transform) const
{
  transform.setIdentity();
}

void FixedJointModel::computeVariablePositions(const Eigen::Isometry3d& /* transform */, double* /* joint_values */) const
{
}
}  // namespace tobas
