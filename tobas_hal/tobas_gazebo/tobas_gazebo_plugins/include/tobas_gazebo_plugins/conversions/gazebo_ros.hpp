#pragma once

#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>

namespace gazebo
{
void timeGazeboToRos(const common::Time& g, ros::Time& r);
void timeRosToGazebo(const ros::Time& r, common::Time& g);

void vectorGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::Vector3& r);
void vectorRosToGazebo(const geometry_msgs::Vector3& r, ignition::math::Vector3d& g);

void pointGazeboToRos(const ignition::math::Vector3d& g, geometry_msgs::Point& r);
void pointRosToGazebo(const geometry_msgs::Point& r, ignition::math::Vector3d& g);

void quaternionGazeboToRos(const ignition::math::Quaterniond& g, geometry_msgs::Quaternion& r);
void quaternionRosToGazebo(const geometry_msgs::Quaternion& r, ignition::math::Quaterniond& g);

void poseGazeboToRos(const ignition::math::Pose3d& g, geometry_msgs::Pose& r);
void poseRosToGazebo(const geometry_msgs::Pose& r, ignition::math::Pose3d& g);
}  // namespace gazebo
