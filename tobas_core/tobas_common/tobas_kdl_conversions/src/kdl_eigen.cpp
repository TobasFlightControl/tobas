// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl_conversions/kdl_eigen.hpp"

namespace tobas
{
namespace kdl
{
void quaternionKDLToEigen(const Quaternion& _k, Eigen::Quaterniond& _e)
{
  _e.x() = _k.x;
  _e.y() = _k.y;
  _e.z() = _k.z;
  _e.w() = _k.w;
}

void quaternionEigenToKDL(const Eigen::Quaterniond& _e, Quaternion& _k)
{
  _k.x = _e.x();
  _k.y = _e.y();
  _k.z = _e.z();
  _k.w = _e.w();
}
}  // namespace kdl
}  // namespace tobas
