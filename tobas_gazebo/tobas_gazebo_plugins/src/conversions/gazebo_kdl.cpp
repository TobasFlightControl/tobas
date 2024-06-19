#include "../../include/tobas_gazebo_plugins/conversions/gazebo_kdl.hpp"

using namespace ignition::math;

namespace gazebo
{
void vectorGazeboToKDL(const ignition::math::Vector3d& g, tobas_kdl::Vector& k)
{
  k.x(g.X());
  k.y(g.Y());
  k.z(g.Z());
}

void vectorKDLToGazebo(const tobas_kdl::Vector& k, ignition::math::Vector3d& g)
{
  g.X(k.x());
  g.Y(k.y());
  g.Z(k.z());
}

void quaternionGazeboToKDL(const ignition::math::Quaterniond& g, tobas_kdl::Quaternion& k)
{
  k.x = g.X();
  k.y = g.Y();
  k.z = g.Z();
  k.w = g.W();
}

void quaternionKDLToGazebo(const tobas_kdl::Quaternion& k, ignition::math::Quaterniond& g)
{
  g.X(k.x);
  g.Y(k.y);
  g.Z(k.z);
  g.W(k.w);
}
}  // namespace gazebo
