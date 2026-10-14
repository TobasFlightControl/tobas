// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace tobas::gazebo
{
void vectorGazeboToRos(const gz::math::Vector3d& _g, geometry_msgs::msg::Vector3& _m);
void vectorRosToGazebo(const geometry_msgs::msg::Vector3& _m, gz::math::Vector3d& _g);

void pointGazeboToRos(const gz::math::Vector3d& _g, geometry_msgs::msg::Point& _m);
void pointRosToGazebo(const geometry_msgs::msg::Point& _m, gz::math::Vector3d& _g);

void quaternionGazeboToRos(const gz::math::Quaterniond& _g, geometry_msgs::msg::Quaternion& _m);
void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& _m, gz::math::Quaterniond& _g);

void poseGazeboToRos(const gz::math::Pose3d& _g, geometry_msgs::msg::Pose& _m);
void poseRosToGazebo(const geometry_msgs::msg::Pose& _m, gz::math::Pose3d& _g);
}  // namespace tobas::gazebo
