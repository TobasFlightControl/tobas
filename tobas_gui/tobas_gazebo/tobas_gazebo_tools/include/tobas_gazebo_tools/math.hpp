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
bool allGreaterEqual(const gz::math::Vector3<T>& _v, T _x)
{
  return _v.X() >= _x && _v.Y() >= _x && _v.Z() >= _x;
}

/**
 * Convert from the FLU (Front-Left-Up) coordinate system used by Gazebo
 * to the FRD (Front-Right-Down) coordinate system used in aerodynamics.
 */
template <typename T>
void FLU2FRD(gz::math::Vector3<T>& _v)
{
  _v.Y() = -_v.Y();
  _v.Z() = -_v.Z();
}

/**
 * Convert from the FRD (Front-Right-Down) coordinate system used in aerodynamics
 * to the FLU (Front-Left-Up) coordinate system used by Gazebo.
 */
template <typename T>
void FRD2FLU(gz::math::Vector3<T>& _v)
{
  _v.Y() = -_v.Y();
  _v.Z() = -_v.Z();
}

/** Create a quaternion from an equivalent angle-axis vector. */
gz::math::Quaterniond quaternionFromAngleAxis(const gz::math::Vector3d& _w);

/** Compute the skew-symmetric matrix representing the cross product of a 3D vector. */
gz::math::Matrix3d skewMatrix(const gz::math::Vector3d& _v);
}  // namespace gazebo
}  // namespace tobas
