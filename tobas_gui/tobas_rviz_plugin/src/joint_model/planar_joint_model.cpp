// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/planar_joint_model.hpp"

#include <limits>

namespace tobas
{
PlanarJointModel::PlanarJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kPlanar;

  for (const auto* variable_name : { "x", "y", "theta" }) {
    variable_names_.push_back(getName() + "/" + variable_name);
  }
}

void PlanarJointModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < 2; ++i) {
    values[i] = 0.0;
  }
  values[2] = 0.0;
}

void PlanarJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  transform = Eigen::Isometry3d(
    Eigen::Translation3d(joint_values[0], joint_values[1], 0.0) *
    Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().x();
  joint_values[1] = transform.translation().y();

  const Eigen::Quaterniond q(transform.linear());

  // Taken from Bullet
  const auto s_squared = 1.0 - (q.w() * q.w());
  if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
    joint_values[2] = 0.0;
  }
  else {
    const auto s = 1.0 / std::sqrt(s_squared);
    joint_values[2] = (std::acos(q.w()) * 2.0f) * (q.z() * s);
  }
}
}  // namespace tobas
