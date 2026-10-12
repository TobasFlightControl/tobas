// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>
#include <gz/math/Vector3.hh>

namespace tobas
{
namespace gazebo
{
template <typename T>
void vector3GazeboToEigen(const gz::math::Vector3<T>& _g, Eigen::Matrix<T, 3, 1>& _e)
{
  _e.x() = _g.X();
  _e.y() = _g.Y();
  _e.z() = _g.Z();
}

template <typename T>
void vector3EigenToGazebo(const Eigen::Matrix<T, 3, 1>& _e, gz::math::Vector3<T>& _g)
{
  _g.X() = _e.x();
  _g.Y() = _e.y();
  _g.Z() = _e.z();
}

template <typename T>
void matrix3EigenToGazebo(const Eigen::Matrix<T, 3, 3>& _e, gz::math::Matrix3<T>& _g)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      _g(r, c) = _e(r, c);
    }
  }
}

template <typename T>
void matrix3EigenToGazebo(const gz::math::Matrix3<T>& _g, Eigen::Matrix<T, 3, 3>& _e)
{
  for (size_t r = 0; r < 3; ++r) {
    for (size_t c = 0; c < 3; ++c) {
      _e(r, c) = _g(r, c);
    }
  }
}
}  // namespace gazebo
}  // namespace tobas
