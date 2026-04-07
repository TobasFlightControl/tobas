// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/revolute_joint_model.hpp"

#include <algorithm>
#include <cmath>

#include <geometric_shapes/check_isometry.h>

#include <tobas_math/definitions.hpp>

namespace tobas
{
RevoluteJointModel::RevoluteJointModel(const std::string& name, size_t joint_index, size_t first_variable_index)
  : JointModel(name, joint_index, first_variable_index)
  , axis_(0., 0., 0.)
  , continuous_(false)
  , x2_(0.)
  , y2_(0.)
  , z2_(0.)
  , xy_(0.)
  , xz_(0.)
  , yz_(0.)
{
  type_ = REVOLUTE;
  variable_names_.push_back(getName());
  variable_index_map_[getName()] = 0;
}

void RevoluteJointModel::setAxis(const Eigen::Vector3d& axis)
{
  axis_ = axis.normalized();
  x2_ = axis_.x() * axis_.x();
  y2_ = axis_.y() * axis_.y();
  z2_ = axis_.z() * axis_.z();
  xy_ = axis_.x() * axis_.y();
  xz_ = axis_.x() * axis_.z();
  yz_ = axis_.y() * axis_.z();
}

void RevoluteJointModel::setContinuous(bool flag)
{
  continuous_ = flag;
}

void RevoluteJointModel::getVariableDefaultPositions(double* values) const
{
  values[0] = 0.;
}

void RevoluteJointModel::computeTransform(const double* joint_values, Eigen::Isometry3d& transform) const
{
  const double c = cos(joint_values[0]);
  const double s = sin(joint_values[0]);
  const double t = 1. - c;
  const double txy = t * xy_;
  const double txz = t * xz_;
  const double tyz = t * yz_;

  const double zs = axis_.z() * s;
  const double ys = axis_.y() * s;
  const double xs = axis_.x() * s;

  // column major
  double* d = transform.data();

  d[0] = t * x2_ + c;
  d[1] = txy + zs;
  d[2] = txz - ys;
  d[3] = 0.;

  d[4] = txy - zs;
  d[5] = t * y2_ + c;
  d[6] = tyz + xs;
  d[7] = 0.;

  d[8] = txz + ys;
  d[9] = tyz - xs;
  d[10] = t * z2_ + c;
  d[11] = 0.;

  d[12] = 0.;
  d[13] = 0.;
  d[14] = 0.;
  d[15] = 1.;

  //  transform = Eigen::Isometry3d(Eigen::AngleAxisd(joint_values[0], axis_));
}

void RevoluteJointModel::computeVariablePositions(const Eigen::Isometry3d& transform, double* joint_values) const
{
  ASSERT_ISOMETRY(transform)  // unsanitized input, could contain a non-isometry
  Eigen::Quaterniond q(transform.linear());
  q.normalize();
  size_t max_idx;
  axis_.array().abs().maxCoeff(&max_idx);
  joint_values[0] = 2. * atan2(q.vec()[max_idx] / axis_[max_idx], q.w());
}
}  // namespace tobas
