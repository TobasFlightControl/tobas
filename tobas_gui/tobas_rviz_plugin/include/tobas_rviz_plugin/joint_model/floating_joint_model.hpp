// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./joint_model.hpp"

namespace tobas
{
/* A floating joint */
class FloatingJointModel : public JointModel
{
public:
  explicit FloatingJointModel(const std::string& name, size_t joint_index, size_t first_variable_index);

  void getVariableDefaultPositions(double* values, const Bounds& other_bounds) const override;
  uint32_t getStateSpaceDimension() const override;
  void computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const override;
  void computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const override;

  double getAngularDistanceWeight() const
  {
    return angular_distance_weight_;
  }

  void setAngularDistanceWeight(double weight)
  {
    angular_distance_weight_ = weight;
  }

  /* Normalize the quaternion (warn if norm is 0, and set to identity); Return true if any change was made. */
  bool normalizeRotation(double* values) const;

  /* Get the distance between the rotation components of two states. */
  double distanceRotation(const double* values1, const double* values2) const;

  /* Get the distance between the translation components of two states. */
  double distanceTranslation(const double* values1, const double* values2) const;

private:
  double angular_distance_weight_;
};
}  // namespace tobas
