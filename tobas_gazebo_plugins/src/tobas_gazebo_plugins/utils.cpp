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
}  // namespace gazebo
