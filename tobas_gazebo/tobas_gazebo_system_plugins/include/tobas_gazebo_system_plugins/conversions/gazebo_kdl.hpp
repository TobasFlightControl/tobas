#pragma once

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_kdl/quaternion.hpp>
#include <tobas_kdl/frame.hpp>

namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& g, kdl::Vector& k);
void vectorKDLToGazebo(const kdl::Vector& k, gz::math::Vector3d& g);

void rotationGazeboToKDL(const gz::math::Quaterniond& g, kdl::Rotation& k);
void rotationKDLToGazebo(const kdl::Rotation& k, gz::math::Quaterniond& g);

void quaternionGazeboToKDL(const gz::math::Quaterniond& g, kdl::Quaternion& k);
void quaternionKDLToGazebo(const kdl::Quaternion& k, gz::math::Quaterniond& g);

void poseGazeboToKDL(const gz::math::Pose3d& g, kdl::Frame& k);
void poseKDLToGazebo(const kdl::Frame& k, gz::math::Pose3d& g);
}  // namespace gazebo
