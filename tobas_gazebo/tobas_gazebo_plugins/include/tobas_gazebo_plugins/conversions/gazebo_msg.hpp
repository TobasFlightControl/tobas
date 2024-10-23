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
void vectorGazeboToMsg(const gz::math::Vector3d& g, geometry_msgs::msg::Vector3& m);
void vectorMsgToGazebo(const geometry_msgs::msg::Vector3& m, gz::math::Vector3d& g);

void pointGazeboToMsg(const gz::math::Vector3d& g, geometry_msgs::msg::Point& m);
void pointMsgToGazebo(const geometry_msgs::msg::Point& m, gz::math::Vector3d& g);

void quaternionGazeboToMsg(const gz::math::Quaterniond& g, geometry_msgs::msg::Quaternion& m);
void quaternionMsgToGazebo(const geometry_msgs::msg::Quaternion& m, gz::math::Quaterniond& g);

void poseGazeboToMsg(const gz::math::Pose3d& g, geometry_msgs::msg::Pose& m);
void poseMsgToGazebo(const geometry_msgs::msg::Pose& m, gz::math::Pose3d& g);
}  // namespace gazebo
