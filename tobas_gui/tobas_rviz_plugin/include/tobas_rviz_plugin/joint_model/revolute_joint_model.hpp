// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
class RevoluteJointModel : public JointModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit RevoluteJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values) const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const override;

  void setAxis(const Eigen::Vector3d& axis);

private:
  Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();  // The axis of the joint
  double x2_ = 0.0, y2_ = 0.0, z2_ = 0.0, xy_ = 0.0, xz_ = 0.0, yz_ = 0.0;
};
}  // namespace tobas
