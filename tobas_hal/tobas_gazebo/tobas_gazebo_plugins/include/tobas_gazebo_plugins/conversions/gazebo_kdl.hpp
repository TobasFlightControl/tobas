#pragma once

#include <gazebo/gazebo.hh>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/quaternion.hpp>

namespace gazebo
{
void vectorGazeboToKDL(const ignition::math::Vector3d& g, KDL::Vector& k);
void vectorKDLToGazebo(const KDL::Vector& k, ignition::math::Vector3d& g);

void quaternionGazeboToKDL(const ignition::math::Quaterniond& g, KDL::Quaternion& k);
void quaternionKDLToGazebo(const KDL::Quaternion& k, ignition::math::Quaterniond& g);
}  // namespace gazebo
