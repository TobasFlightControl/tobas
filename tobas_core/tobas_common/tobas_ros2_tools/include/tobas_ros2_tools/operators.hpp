// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace geometry_msgs::msg
{
Vector3 operator*(double _lhs, const Vector3& _rhs);
Vector3 operator*(const Vector3& _lhs, double _rhs);
Vector3 operator/(double _lhs, const Vector3& _rhs);
Vector3 operator/(const Vector3& _lhs, double _rhs);

Vector3 operator+(const Vector3& _lhs, const Vector3& _rhs);
Vector3 operator-(const Vector3& _lhs, const Vector3& _rhs);

Vector3 operator-(const Point& _lhs, const Point& _rhs);
}  // namespace geometry_msgs::msg
