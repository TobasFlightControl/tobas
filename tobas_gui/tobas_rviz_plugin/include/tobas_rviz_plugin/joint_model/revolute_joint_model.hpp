// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A revolute joint */
class RevoluteJointModel : public JointModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit RevoluteJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values) const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const override;

  void setContinuous(bool flag);

  /* Check if this joint wraps around */
  bool isContinuous() const
  {
    return continuous_;
  }

  /* Get the axis of rotation */
  const Eigen::Vector3d& getAxis() const
  {
    return axis_;
  }

  /* Set the axis of rotation */
  void setAxis(const Eigen::Vector3d& axis);

protected:
  /* The axis of the joint */
  Eigen::Vector3d axis_;

  /* Flag indicating whether this joint wraps around */
  bool continuous_;

private:
  double x2_, y2_, z2_, xy_, xz_, yz_;
};
}  // namespace tobas
