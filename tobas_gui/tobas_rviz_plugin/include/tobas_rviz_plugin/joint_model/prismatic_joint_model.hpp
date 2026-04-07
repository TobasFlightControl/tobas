// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A prismatic joint */
class PrismaticJointModel : public JointModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PrismaticJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values) const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const override;

  /* Get the axis of translation */
  const Eigen::Vector3d& getAxis() const
  {
    return axis_;
  }

  /* Set the axis of translation */
  void setAxis(const Eigen::Vector3d& axis)
  {
    axis_ = axis;
  }

protected:
  /* The axis of the joint */
  Eigen::Vector3d axis_;
};
}  // namespace tobas
