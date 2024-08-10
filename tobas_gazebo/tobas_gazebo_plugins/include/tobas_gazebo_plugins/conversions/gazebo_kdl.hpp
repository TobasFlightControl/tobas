#pragma once

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/quaternion.hpp>

namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& g, kdl::Vector& k);
void vectorKDLToGazebo(const kdl::Vector& k, gz::math::Vector3d& g);

void quaternionGazeboToKDL(const gz::math::Quaterniond& g, kdl::Quaternion& k);
void quaternionKDLToGazebo(const kdl::Quaternion& k, gz::math::Quaterniond& g);
}  // namespace gazebo
