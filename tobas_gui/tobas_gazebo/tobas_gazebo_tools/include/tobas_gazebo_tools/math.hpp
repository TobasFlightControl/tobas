// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

namespace tobas
{
namespace gazebo
{
template <typename T>
bool allGreaterEqual(const gz::math::Vector3<T>& v, T x)
{
  return v.X() >= x && v.Y() >= x && v.Z() >= x;
}

/**
 * @brief Convert from the FLU (Front-Left-Up) coordinate system used by Gazebo
 * to the FRD (Front-Right-Down) coordinate system used in aerodynamics.
 */
template <typename T>
void FLU2FRD(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/**
 * @brief Convert from the FRD (Front-Right-Down) coordinate system used in aerodynamics
 * to the FLU (Front-Left-Up) coordinate system used by Gazebo.
 */
template <typename T>
void FRD2FLU(gz::math::Vector3<T>& v)
{
  v.Y() = -v.Y();
  v.Z() = -v.Z();
}

/* Create a quaternion from an equivalent angle-axis vector. */
gz::math::Quaterniond quaternionFromAngleAxis(const gz::math::Vector3d& w);

/* Compute the skew-symmetric matrix representing the cross product of a 3D vector. */
gz::math::Matrix3d skewMatrix(const gz::math::Vector3d& v);
}  // namespace gazebo
}  // namespace tobas
