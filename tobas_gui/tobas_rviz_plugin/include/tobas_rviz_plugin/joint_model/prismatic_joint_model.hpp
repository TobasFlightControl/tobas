// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
namespace rviz
{
class PrismaticJointModel : public JointModel
{
public:
  explicit PrismaticJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index);

  void getVariableDefaultPositions(double* _values) const override;
  void computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const override;

  void setAxis(const Eigen::Vector3d& _axis);

private:
  Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();  ///< The axis of the joint
};
}  // namespace rviz
}  // namespace tobas
