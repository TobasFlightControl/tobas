// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/prismatic_joint_model.hpp"

#include <limits>

namespace tobas
{
PrismaticJointModel::PrismaticJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index), axis_(0., 0., 0.)
{
  type_ = PRISMATIC;
  variable_names_.push_back(getName());
  variable_bounds_.resize(1);
  variable_bounds_[0].position_bounded_ = true;
  variable_bounds_[0].min_position_ = -std::numeric_limits<double>::max();
  variable_bounds_[0].max_position_ = std::numeric_limits<double>::max();
  variable_index_map_[getName()] = 0;
  computeVariableBoundsMsg();
}

uint32_t PrismaticJointModel::getStateSpaceDimension() const
{
  return 1;
}

void PrismaticJointModel::getVariableDefaultPositions(double* values, const Bounds& bounds) const
{
  // if zero is a valid value
  if (bounds[0].min_position_ <= 0. && bounds[0].max_position_ >= 0.) {
    values[0] = 0.;
  }
  else {
    values[0] = (bounds[0].min_position_ + bounds[0].max_position_) / 2.;
  }
}

void PrismaticJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  double* d = transform.data();
  d[0] = 1.;
  d[1] = 0.;
  d[2] = 0.;
  d[3] = 0.;

  d[4] = 0.;
  d[5] = 1.;
  d[6] = 0.;
  d[7] = 0.;

  d[8] = 0.;
  d[9] = 0.;
  d[10] = 1.;
  d[11] = 0.;

  d[12] = axis_.x() * joint_values[0];
  d[13] = axis_.y() * joint_values[0];
  d[14] = axis_.z() * joint_values[0];
  d[15] = 1.;

  //  transform.setIdentity();
  //  transform.translation() = Eigen::Vector3d(axis_ * joint_values[0]);
}

void PrismaticJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  joint_values[0] = transform.translation().dot(axis_);
}
}  // namespace tobas
