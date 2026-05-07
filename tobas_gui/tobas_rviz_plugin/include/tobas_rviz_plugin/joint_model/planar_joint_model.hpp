// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
class PlanarJointModel : public JointModel
{
public:
  explicit PlanarJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values) const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const override;
};
}  // namespace tobas
