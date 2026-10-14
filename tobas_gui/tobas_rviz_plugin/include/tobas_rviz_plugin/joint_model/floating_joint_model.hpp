// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas::rviz
{
class FloatingJointModel : public JointModel
{
public:
  explicit FloatingJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index);

  void getVariableDefaultPositions(double* _values) const override;
  void computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const override;
};
}  // namespace tobas::rviz
