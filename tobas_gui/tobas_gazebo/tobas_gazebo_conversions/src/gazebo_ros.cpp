// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_conversions/gazebo_ros.hpp"

namespace tobas
{
namespace gazebo
{
void vectorGazeboToRos(const gz::math::Vector3d& _g, geometry_msgs::msg::Vector3& _m)
{
  _m.x = _g.X();
  _m.y = _g.Y();
  _m.z = _g.Z();
}

void vectorRosToGazebo(const geometry_msgs::msg::Vector3& _m, gz::math::Vector3d& _g)
{
  _g.Set(_m.x, _m.y, _m.z);
}

void pointGazeboToRos(const gz::math::Vector3d& _g, geometry_msgs::msg::Point& _m)
{
  _m.x = _g.X();
  _m.y = _g.Y();
  _m.z = _g.Z();
}

void pointRosToGazebo(const geometry_msgs::msg::Point& _m, gz::math::Vector3d& _g)
{
  _g.Set(_m.x, _m.y, _m.z);
}

void quaternionGazeboToRos(const gz::math::Quaterniond& _g, geometry_msgs::msg::Quaternion& _m)
{
  _m.w = _g.W();
  _m.x = _g.X();
  _m.y = _g.Y();
  _m.z = _g.Z();
}

void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& _m, gz::math::Quaterniond& _g)
{
  _g.Set(_m.w, _m.x, _m.y, _m.z);
}

void poseGazeboToRos(const gz::math::Pose3d& _g, geometry_msgs::msg::Pose& _m)
{
  pointGazeboToRos(_g.Pos(), _m.position);
  quaternionGazeboToRos(_g.Rot(), _m.orientation);
}

void poseRosToGazebo(const geometry_msgs::msg::Pose& _m, gz::math::Pose3d& _g)
{
  pointRosToGazebo(_m.position, _g.Pos());
  quaternionRosToGazebo(_m.orientation, _g.Rot());
}
}  // namespace gazebo
}  // namespace tobas
