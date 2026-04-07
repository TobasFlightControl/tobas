// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/planar_joint_model.hpp"

#include <limits>

#include <geometric_shapes/check_isometry.h>

namespace tobas
{
PlanarJointModel::PlanarJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
{
  type_ = kPlanar;

  local_variable_names_.push_back("x");
  local_variable_names_.push_back("y");
  local_variable_names_.push_back("theta");

  for (size_t i = 0; i < 3; ++i) {
    variable_names_.push_back(getName() + "/" + local_variable_names_[i]);
    variable_index_map_[variable_names_.back()] = i;
  }
}

void PlanarJointModel::getVariableDefaultPositions(double* values) const
{
  for (size_t i = 0; i < 2; ++i) {
    values[i] = 0.;
  }
  values[2] = 0.;
}

void PlanarJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  transform = Eigen::Isometry3d(
    Eigen::Translation3d(joint_values[0], joint_values[1], 0.) *
    Eigen::AngleAxisd(joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().x();
  joint_values[1] = transform.translation().y();

  ASSERT_ISOMETRY(transform)  // Unsanitized input, could contain a non-isometry
  const Eigen::Quaterniond q(transform.linear());

  // Taken from Bullet
  const auto s_squared = 1. - (q.w() * q.w());
  if (s_squared < 10. * std::numeric_limits<double>::epsilon()) {
    joint_values[2] = 0.;
  }
  else {
    const auto s = 1. / sqrt(s_squared);
    joint_values[2] = (acos(q.w()) * 2.0f) * (q.z() * s);
  }
}
}  // namespace tobas
