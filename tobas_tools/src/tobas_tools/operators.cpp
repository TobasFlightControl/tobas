#include "../../include/tobas_tools/operators.hpp"

using namespace geometry_msgs;

Vector3 operator*(const double& lhs, const Vector3& rhs)
{
  Vector3 res;
  res.x = lhs * rhs.x;
  res.y = lhs * rhs.y;
  res.z = lhs * rhs.z;
  return res;
}

Vector3 operator*(const Vector3& lhs, const double& rhs)
{
  Vector3 res;
  res.x = lhs.x * rhs;
  res.y = lhs.y * rhs;
  res.z = lhs.z * rhs;
  return res;
}

Vector3 operator+(const Vector3& lhs, const Vector3& rhs)
{
  Vector3 res;
  res.x = lhs.x + rhs.x;
  res.y = lhs.y + rhs.y;
  res.z = lhs.z + rhs.z;
  return res;
}

Vector3 operator-(const Vector3& lhs, const Vector3& rhs)
{
  Vector3 res;
  res.x = lhs.x - rhs.x;
  res.y = lhs.y - rhs.y;
  res.z = lhs.z - rhs.z;
  return res;
}

Vector3 operator-(const Point& lhs, const Point& rhs)
{
  Vector3 res;
  res.x = lhs.x - rhs.x;
  res.y = lhs.y - rhs.y;
  res.z = lhs.z - rhs.z;
  return res;
}

Vector3 operator-(const tobas_msgs::LinearVelocity& lhs, const tobas_msgs::LinearVelocity& rhs)
{
  Vector3 res;
  res.x = lhs.vx - rhs.vx;
  res.y = lhs.vy - rhs.vy;
  res.z = lhs.vz - rhs.vz;
  return res;
}
