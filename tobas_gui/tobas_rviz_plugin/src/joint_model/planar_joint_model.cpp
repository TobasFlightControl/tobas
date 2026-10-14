// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/planar_joint_model.hpp"

#include <limits>

namespace tobas::rviz
{
PlanarJointModel::PlanarJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kPlanar;

  for (const auto* variable_name : { "x", "y", "theta" }) {
    variable_names_.push_back(getName() + "/" + variable_name);
  }
}

void PlanarJointModel::getVariableDefaultPositions(double* _values) const
{
  for (size_t i = 0; i < 2; ++i) {
    _values[i] = 0.0;
  }
  _values[2] = 0.0;
}

void PlanarJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  _transform = Eigen::Isometry3d(
    Eigen::Translation3d(_joint_values[0], _joint_values[1], 0.0) *
    Eigen::AngleAxisd(_joint_values[2], Eigen::Vector3d::UnitZ()));
}

void PlanarJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  _joint_values[0] = _transform.translation().x();
  _joint_values[1] = _transform.translation().y();

  const Eigen::Quaterniond q(_transform.linear());

  // Taken from Bullet
  const auto s_squared = 1.0 - (q.w() * q.w());
  if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
    _joint_values[2] = 0.0;
  }
  else {
    const auto s = 1.0 / std::sqrt(s_squared);
    _joint_values[2] = (std::acos(q.w()) * 2.0f) * (q.z() * s);
  }
}
}  // namespace tobas::rviz
