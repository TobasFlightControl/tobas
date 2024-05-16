#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>

#include <tobas_kdl_msgs/Vector.h>
#include <tobas_kdl_msgs/Twist.h>
#include <tobas_kdl_msgs/Frame.h>
#include <tobas_kdl_msgs/Quaternion.h>

namespace tobas_kdl
{
void vectorKDLToMsg(const Vector& k, geometry_msgs::Vector3& m);
void vectorMsgToKDL(const geometry_msgs::Vector3& m, Vector& k);

void pointKDLToMsg(const Vector& k, geometry_msgs::Point& m);
void pointMsgToKDL(const geometry_msgs::Point& m, Vector& k);

void twistKDLToMsg(const Twist& k, geometry_msgs::Twist& m);
void twistMsgToKDL(const geometry_msgs::Twist& m, Twist& k);

void rotationKDLToMsg(const Rotation& k, geometry_msgs::Quaternion& m);
void rotationMsgToKDL(const geometry_msgs::Quaternion& m, Rotation& k);

void quaternionKDLToMsg(const Quaternion& k, geometry_msgs::Quaternion& m);
void quaternionMsgToKDL(const geometry_msgs::Quaternion& m, Quaternion& k);

void poseKDLToMsg(const Frame& k, geometry_msgs::Pose& m);
void poseMsgToKDL(const geometry_msgs::Pose& m, Frame& k);

void transformKDLToMsg(const Frame& k, geometry_msgs::Transform& m);
void transformMsgToKDL(const geometry_msgs::Transform& m, Frame& k);
}  // namespace tobas_kdl
