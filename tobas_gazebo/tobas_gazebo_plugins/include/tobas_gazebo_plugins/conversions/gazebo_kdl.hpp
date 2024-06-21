#pragma once

#include <gazebo/gazebo.hh>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/quaternion.hpp>

namespace gazebo
{
void vectorGazeboToKDL(const ignition::math::Vector3d& g, kdl::Vector& k);
void vectorKDLToGazebo(const kdl::Vector& k, ignition::math::Vector3d& g);

void quaternionGazeboToKDL(const ignition::math::Quaterniond& g, kdl::Quaternion& k);
void quaternionKDLToGazebo(const kdl::Quaternion& k, ignition::math::Quaterniond& g);
}  // namespace gazebo
