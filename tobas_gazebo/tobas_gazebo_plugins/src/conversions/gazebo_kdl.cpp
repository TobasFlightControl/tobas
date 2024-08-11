#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace gz::math;

namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& g, kdl::Vector& k)
{
  k.x(g.X());
  k.y(g.Y());
  k.z(g.Z());
}

void vectorKDLToGazebo(const kdl::Vector& k, gz::math::Vector3d& g)
{
  g.X(k.x());
  g.Y(k.y());
  g.Z(k.z());
}

void quaternionGazeboToKDL(const gz::math::Quaterniond& g, kdl::Quaternion& k)
{
  k.x = g.X();
  k.y = g.Y();
  k.z = g.Z();
  k.w = g.W();
}

void quaternionKDLToGazebo(const kdl::Quaternion& k, gz::math::Quaterniond& g)
{
  g.SetX(k.x);
  g.SetY(k.y);
  g.SetZ(k.z);
  g.SetW(k.w);
}
}  // namespace gazebo
