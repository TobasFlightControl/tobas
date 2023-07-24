#include "../../include/tobas_gazebo_plugins/utils.hpp"

using namespace ignition::math;

namespace gazebo
{
Quaterniond angleAxisToQuaternion(const Vector3d& w)
{
  const auto angle = w.Length();
  if (angle < 1e-9)
  {
    return Quaterniond::Identity;
  }

  const auto axis = w.Normalized();
  const auto mag = sin(angle / 2.);
  return Quaterniond(cos(angle / 2.), mag * axis.X(), mag * axis.Y(), mag * axis.Z());
}

Matrix3d skewMatrix(const Vector3d& v)
{
  Matrix3d res;
  res(0, 0) = 0.;
  res(0, 1) = -v.Z();
  res(0, 2) = v.Y();
  res(1, 0) = v.Z();
  res(1, 1) = 0.;
  res(1, 2) = -v.X();
  res(2, 0) = -v.Y();
  res(2, 1) = v.X();
  res(2, 2) = 0.;
  return res;
}\
}  // namespace gazebo
