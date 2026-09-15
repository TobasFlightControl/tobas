// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl_conversions/kdl_eigen.hpp"

namespace tobas
{
namespace kdl
{
void quaternionKDLToEigen(const Quaternion& k, Eigen::Quaterniond& e)
{
  e.x() = k.x;
  e.y() = k.y;
  e.z() = k.z;
  e.w() = k.w;
}

void quaternionEigenToKDL(const Eigen::Quaterniond& e, Quaternion& k)
{
  k.x = e.x();
  k.y = e.y();
  k.z = e.z();
  k.w = e.w();
}
}  // namespace kdl
}  // namespace tobas
