// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_tools/math.hpp"

namespace tobas
{
namespace gazebo
{
gz::math::Quaterniond quaternionFromAngleAxis(const gz::math::Vector3d& w)
{
  const auto angle = w.Length();
  if (angle < 1e-9) {
    return gz::math::Quaterniond::Identity;
  }

  const auto axis = w.Normalized();
  const auto mag = std::sin(angle / 2);
  return gz::math::Quaterniond(std::cos(angle / 2), mag * axis.X(), mag * axis.Y(), mag * axis.Z());
}

gz::math::Matrix3d skewMatrix(const gz::math::Vector3d& v)
{
  gz::math::Matrix3d res;
  res(0, 0) = 0;
  res(0, 1) = -v.Z();
  res(0, 2) = v.Y();
  res(1, 0) = v.Z();
  res(1, 1) = 0;
  res(1, 2) = -v.X();
  res(2, 0) = -v.Y();
  res(2, 1) = v.X();
  res(2, 2) = 0;
  return res;
}
}  // namespace gazebo
}  // namespace tobas
