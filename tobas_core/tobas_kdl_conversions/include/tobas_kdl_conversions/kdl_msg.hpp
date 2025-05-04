#pragma once

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl/twist.hpp>
#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/quaternion.hpp>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>

namespace kdl
{
void vectorKDLToMsg(const Vector& k, geometry_msgs::msg::Vector3& m);
void vectorMsgToKDL(const geometry_msgs::msg::Vector3& m, Vector& k);

void pointKDLToMsg(const Vector& k, geometry_msgs::msg::Point& m);
void pointMsgToKDL(const geometry_msgs::msg::Point& m, Vector& k);

void twistKDLToMsg(const Twist& k, geometry_msgs::msg::Twist& m);
void twistMsgToKDL(const geometry_msgs::msg::Twist& m, Twist& k);

void rotationKDLToMsg(const Rotation& k, geometry_msgs::msg::Quaternion& m);
void rotationMsgToKDL(const geometry_msgs::msg::Quaternion& m, Rotation& k);

void quaternionKDLToMsg(const Quaternion& k, geometry_msgs::msg::Quaternion& m);
void quaternionMsgToKDL(const geometry_msgs::msg::Quaternion& m, Quaternion& k);

void poseKDLToMsg(const Frame& k, geometry_msgs::msg::Pose& m);
void poseMsgToKDL(const geometry_msgs::msg::Pose& m, Frame& k);

void transformKDLToMsg(const Frame& k, geometry_msgs::msg::Transform& m);
void transformMsgToKDL(const geometry_msgs::msg::Transform& m, Frame& k);
}  // namespace kdl
