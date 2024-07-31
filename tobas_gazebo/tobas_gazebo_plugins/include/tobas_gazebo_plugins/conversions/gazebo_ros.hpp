#pragma once

#include <rclcpp/rclcpp.hpp>
#include <gazebo/gazebo.hh>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/Pose.h>

namespace gazebo
{
void timeGazeboToRos(const common::Time& g, rclcpp::Time& r);
void timeRosToGazebo(const rclcpp::Time& r, common::Time& g);

void vectorGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::msg::Vector3& r);
void vectorRosToGazebo(const geometry_msgs::msg::Vector3& r, ignition::math::Vector3d& g);

void pointGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::msg::Point& r);
void pointRosToGazebo(const geometry_msgs::msg::Point& r, ignition::math::Vector3d& g);

void quaternionGazeboToRos(const ignition::math::Quaterniond& g, geometry_msgs::msg::Quaternion& r);
void quaternionRosToGazebo(const geometry_msgs::msg::Quaternion& r, ignition::math::Quaterniond& g);

void poseGazeboToRos(const ignition::math::Pose3d& g, geometry_msgs::msg::Pose& r);
void poseRosToGazebo(const geometry_msgs::msg::Pose& r, ignition::math::Pose3d& g);
}  // namespace gazebo
