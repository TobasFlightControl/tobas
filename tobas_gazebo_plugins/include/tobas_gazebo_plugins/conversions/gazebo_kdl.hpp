#pragma once

#include <gazebo/gazebo.hh>

#include <dh_kdl/frames.hpp>

namespace gazebo
{
template <typename T>
void vectorGazeboToKDL(const ignition::math::Vector3<T>& g, KDL::Vector& k)
{
  k.x(g.X());
  k.y(g.Y());
  k.z(g.Z());
}

template <typename T>
void vectorKDLToGazebo(const KDL::Vector& k, ignition::math::Vector3<T>& g)
{
  g.X() = k.x();
  g.Y() = k.y();
  g.Z() = k.z();
}
}  // namespace gazebo
