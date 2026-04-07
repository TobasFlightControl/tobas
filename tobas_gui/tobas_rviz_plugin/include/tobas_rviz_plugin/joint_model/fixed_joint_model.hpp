// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A fixed joint */
class FixedJointModel : public JointModel
{
public:
  explicit FixedJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const override;
  uint32_t getStateSpaceDimension() const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transf) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transf, double* joint_values) const override;
};
}  // namespace tobas
