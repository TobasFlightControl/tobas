#pragma once

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace geometry_msgs
{
namespace msg
{
Vector3 operator*(const double& lhs, const Vector3& rhs);
Vector3 operator*(const Vector3& lhs, const double& rhs);
Vector3 operator/(const double& lhs, const Vector3& rhs);
Vector3 operator/(const Vector3& lhs, const double& rhs);

Vector3 operator+(const Vector3& lhs, const Vector3& rhs);
Vector3 operator-(const Vector3& lhs, const Vector3& rhs);

Vector3 operator-(const Point& lhs, const Point& rhs);
}  // namespace msg
}  // namespace geometry_msgs
