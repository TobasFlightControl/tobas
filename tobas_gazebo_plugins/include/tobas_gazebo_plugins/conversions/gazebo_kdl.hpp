#pragma once

#include <kdl/frames.hpp>
#include <gazebo/gazebo.hh>

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
