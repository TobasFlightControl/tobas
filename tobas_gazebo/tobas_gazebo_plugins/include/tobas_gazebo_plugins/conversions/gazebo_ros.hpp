#pragma once

#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Pose3.hh>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

namespace gazebo
{
void vectorGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Vector3& r);
void vectorRosToGazebo(const geometry_msgs::msg::Vector3& r, gz::math::Vector3d& g);

void pointGazeboToRos(const gz::math::Vector3d& g, geometry_msgs::msg::Point& r);
void pointRosToGazebo(const geometry_msgs::msg::Point& r, gz::math::Vector3d& g);

void quaternionGazeboToRos(const gz::math::Quaterniond& g, geometry_msgs::msg::Quaternion& r);
void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& r, gz::math::Quaterniond& g);

void poseGazeboToRos(const gz::math::Pose3d& g, geometry_msgs::msg::Pose& r);
void poseRosToGazebo(const geometry_msgs::msg::Pose& r, gz::math::Pose3d& g);
}  // namespace gazebo
