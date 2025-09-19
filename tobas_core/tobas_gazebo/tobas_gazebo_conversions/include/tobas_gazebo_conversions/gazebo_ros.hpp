#pragma once

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace gazebo
{
void vectorGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Vector3& m);
void vectorRosToGazebo(const geometry_msgs::msg::Vector3& m, gz::math::Vector3d& g);

void pointGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Point& m);
void pointRosToGazebo(const geometry_msgs::msg::Point& m, gz::math::Vector3d& g);

void quaternionGazeboToRos(const gz::math::Quaterniond& g, geometry_msgs::msg::Quaternion& m);
void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& m, gz::math::Quaterniond& g);

void poseGazeboToRos(const gz::math::Pose3d& g, geometry_msgs::msg::Pose& m);
void poseRosToGazebo(const geometry_msgs::msg::Pose& m, gz::math::Pose3d& g);
}  // namespace gazebo
