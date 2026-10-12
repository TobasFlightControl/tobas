// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/revolute_joint_model.hpp"

namespace tobas
{
namespace rviz
{
RevoluteJointModel::RevoluteJointModel(const std::string& _name, size_t _joint_index, size_t _first_variable_index)
  : JointModel(_name, _joint_index, _first_variable_index)
{
  type_ = kRevolute;
  variable_names_.push_back(getName());
}

void RevoluteJointModel::getVariableDefaultPositions(double* _values) const
{
  _values[0] = 0.0;
}

void RevoluteJointModel::computeTransform(const double* _joint_values, Eigen::Isometry3d& _transform) const
{
  const auto c = std::cos(_joint_values[0]);
  const auto s = std::sin(_joint_values[0]);
  const auto t = 1.0 - c;
  const auto txy = t * xy_;
  const auto txz = t * xz_;
  const auto tyz = t * yz_;

  const auto zs = axis_.z() * s;
  const auto ys = axis_.y() * s;
  const auto xs = axis_.x() * s;

  // column major
  auto d = _transform.data();

  d[0] = t * x2_ + c;
  d[1] = txy + zs;
  d[2] = txz - ys;
  d[3] = 0.0;

  d[4] = txy - zs;
  d[5] = t * y2_ + c;
  d[6] = tyz + xs;
  d[7] = 0.0;

  d[8] = txz + ys;
  d[9] = tyz - xs;
  d[10] = t * z2_ + c;
  d[11] = 0.0;

  d[12] = 0.0;
  d[13] = 0.0;
  d[14] = 0.0;
  d[15] = 1.0;
}

void RevoluteJointModel::computeVariablePositions(const Eigen::Isometry3d& _transform, double* _joint_values) const
{
  Eigen::Quaterniond q(_transform.linear());
  q.normalize();
  size_t max_idx;
  axis_.array().abs().maxCoeff(&max_idx);
  _joint_values[0] = 2.0 * std::atan2(q.vec()[max_idx] / axis_[max_idx], q.w());
}

void RevoluteJointModel::setAxis(const Eigen::Vector3d& _axis)
{
  axis_ = _axis.normalized();
  x2_ = axis_.x() * axis_.x();
  y2_ = axis_.y() * axis_.y();
  z2_ = axis_.z() * axis_.z();
  xy_ = axis_.x() * axis_.y();
  xz_ = axis_.x() * axis_.z();
  yz_ = axis_.y() * axis_.z();
}
}  // namespace rviz
}  // namespace tobas
