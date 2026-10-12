// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/quaternion.hpp>
#include <tobas_kdl/rotation.hpp>
#include <tobas_kdl/vector.hpp>

namespace tobas
{
namespace gazebo
{
void vectorGazeboToKDL(const gz::math::Vector3d& _g, kdl::Vector& _k);
void vectorKDLToGazebo(const kdl::Vector& _k, gz::math::Vector3d& _g);

void rotationGazeboToKDL(const gz::math::Quaterniond& _g, kdl::Rotation& _k);
void rotationKDLToGazebo(const kdl::Rotation& _k, gz::math::Quaterniond& _g);

void quaternionGazeboToKDL(const gz::math::Quaterniond& _g, kdl::Quaternion& _k);
void quaternionKDLToGazebo(const kdl::Quaternion& _k, gz::math::Quaterniond& _g);

void poseGazeboToKDL(const gz::math::Pose3d& _g, kdl::Frame& _k);
void poseKDLToGazebo(const kdl::Frame& _k, gz::math::Pose3d& _g);
}  // namespace gazebo
}  // namespace tobas
