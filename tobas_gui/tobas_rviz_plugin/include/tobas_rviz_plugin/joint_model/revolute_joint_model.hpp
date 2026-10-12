// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
namespace rviz
{
class RevoluteJointModel : public JointModel
{
public:
  explicit RevoluteJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index);

  void getVariableDefaultPositions(double* _values) const override;
  void computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const override;

  void setAxis(const Eigen::Vector3d& _axis);

private:
  Eigen::Vector3d axis_ = Eigen::Vector3d::Zero();  ///< The axis of the joint
  double x2_ = 0.0, y2_ = 0.0, z2_ = 0.0, xy_ = 0.0, xz_ = 0.0, yz_ = 0.0;
};
}  // namespace rviz
}  // namespace tobas
