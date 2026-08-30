// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/fixed_joint_model.hpp"

namespace tobas
{
namespace rviz
{
FixedJointModel::FixedJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kFixed;
}

void FixedJointModel::getVariableDefaultPositions(double*) const
{
}

void FixedJointModel::computeTransform(const double*, Eigen::Isometry3d& transform) const
{
  transform.setIdentity();
}

void FixedJointModel::computeVariablePositions(const Eigen::Isometry3d&, double*) const
{
}
}  // namespace rviz
}  // namespace tobas
