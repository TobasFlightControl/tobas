// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/operators.hpp"

namespace geometry_msgs
{
namespace msg
{
Vector3 operator*(double _lhs, const Vector3& _rhs)
{
  Vector3 res;
  res.x = _lhs * _rhs.x;
  res.y = _lhs * _rhs.y;
  res.z = _lhs * _rhs.z;
  return res;
}

Vector3 operator*(const Vector3& _lhs, double _rhs)
{
  Vector3 res;
  res.x = _lhs.x * _rhs;
  res.y = _lhs.y * _rhs;
  res.z = _lhs.z * _rhs;
  return res;
}

Vector3 operator/(double _lhs, const Vector3& _rhs)
{
  Vector3 res;
  res.x = _lhs / _rhs.x;
  res.y = _lhs / _rhs.y;
  res.z = _lhs / _rhs.z;
  return res;
}

Vector3 operator/(const Vector3& _lhs, double _rhs)
{
  Vector3 res;
  res.x = _lhs.x / _rhs;
  res.y = _lhs.y / _rhs;
  res.z = _lhs.z / _rhs;
  return res;
}

Vector3 operator+(const Vector3& _lhs, const Vector3& _rhs)
{
  Vector3 res;
  res.x = _lhs.x + _rhs.x;
  res.y = _lhs.y + _rhs.y;
  res.z = _lhs.z + _rhs.z;
  return res;
}

Vector3 operator-(const Vector3& _lhs, const Vector3& _rhs)
{
  Vector3 res;
  res.x = _lhs.x - _rhs.x;
  res.y = _lhs.y - _rhs.y;
  res.z = _lhs.z - _rhs.z;
  return res;
}

Vector3 operator-(const Point& _lhs, const Point& _rhs)
{
  Vector3 res;
  res.x = _lhs.x - _rhs.x;
  res.y = _lhs.y - _rhs.y;
  res.z = _lhs.z - _rhs.z;
  return res;
}
}  // namespace msg
}  // namespace geometry_msgs
