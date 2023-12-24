#pragma once

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include "../frames.hpp"
#include "../quaternion.hpp"

namespace KDL
{
void vectorKDLToMsg(const Vector& k, geometry_msgs::Vector3& m);
void vectorMsgToKDL(const geometry_msgs::Vector3& m, Vector& k);

void pointKDLToMsg(const Vector& k, geometry_msgs::Point& m);
void pointMsgToKDL(const geometry_msgs::Point& m, Vector& k);

void twistKDLToMsg(const Twist& k, geometry_msgs::Twist& m);
void twistMsgToKDL(const geometry_msgs::Twist& m, Twist& k);

void quaternionKDLToMsg(const Quaternion& k, geometry_msgs::Quaternion& m);
void quaternionMsgToKDL(const geometry_msgs::Quaternion& m, Quaternion& k);
}  // namespace KDL
