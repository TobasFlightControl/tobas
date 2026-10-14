// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/fixed_joint_model.hpp"

namespace tobas
{
namespace rviz
{
FixedJointModel::FixedJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kFixed;
}

void FixedJointModel::getVariableDefaultPositions(double*) const
{
}

void FixedJointModel::computeTransform(const double*, Eigen::Isometry3d& _transform) const
{
  _transform.setIdentity();
}

void FixedJointModel::computeVariablePositions(const Eigen::Isometry3d&, double*) const
{
}
}  // namespace rviz
}  // namespace tobas
