// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_kdl_conversions/kdl_msg.hpp"

namespace tobas::kdl
{
void vectorKDLToMsg(const Vector& _k, geometry_msgs::msg::Vector3& _m)
{
  _m.x = _k.x();
  _m.y = _k.y();
  _m.z = _k.z();
}

void vectorMsgToKDL(const geometry_msgs::msg::Vector3& _m, Vector& _k)
{
  _k.x(_m.x);
  _k.y(_m.y);
  _k.z(_m.z);
}

void pointKDLToMsg(const Vector& _k, geometry_msgs::msg::Point& _m)
{
  _m.x = _k.x();
  _m.y = _k.y();
  _m.z = _k.z();
}

void pointMsgToKDL(const geometry_msgs::msg::Point& _m, Vector& _k)
{
  _k.x(_m.x);
  _k.y(_m.y);
  _k.z(_m.z);
}

void twistKDLToMsg(const Twist& _k, geometry_msgs::msg::Twist& _m)
{
  vectorKDLToMsg(_k.vel, _m.linear);
  vectorKDLToMsg(_k.rot, _m.angular);
}

void twistMsgToKDL(const geometry_msgs::msg::Twist& _m, Twist& _k)
{
  vectorMsgToKDL(_m.linear, _k.vel);
  vectorMsgToKDL(_m.angular, _k.rot);
}

void rotationKDLToMsg(const Rotation& _k, geometry_msgs::msg::Quaternion& _m)
{
  _k.getQuaternion(_m.x, _m.y, _m.z, _m.w);
}

void rotationMsgToKDL(const geometry_msgs::msg::Quaternion& _m, Rotation& _k)
{
  _k = Rotation::Quaternion(_m.x, _m.y, _m.z, _m.w);
}

void quaternionKDLToMsg(const Quaternion& _k, geometry_msgs::msg::Quaternion& _m)
{
  _m.x = _k.x;
  _m.y = _k.y;
  _m.z = _k.z;
  _m.w = _k.w;
}

void quaternionMsgToKDL(const geometry_msgs::msg::Quaternion& _m, Quaternion& _k)
{
  _k.x = _m.x;
  _k.y = _m.y;
  _k.z = _m.z;
  _k.w = _m.w;
}

void poseKDLToMsg(const Frame& _k, geometry_msgs::msg::Pose& _m)
{
  pointKDLToMsg(_k.p, _m.position);
  rotationKDLToMsg(_k.M, _m.orientation);
}

void poseMsgToKDL(const geometry_msgs::msg::Pose& _m, Frame& _k)
{
  pointMsgToKDL(_m.position, _k.p);
  rotationMsgToKDL(_m.orientation, _k.M);
}

void transformKDLToMsg(const Frame& _k, geometry_msgs::msg::Transform& _m)
{
  vectorKDLToMsg(_k.p, _m.translation);
  rotationKDLToMsg(_k.M, _m.rotation);
}

void transformMsgToKDL(const geometry_msgs::msg::Transform& _m, Frame& _k)
{
  vectorMsgToKDL(_m.translation, _k.p);
  rotationMsgToKDL(_m.rotation, _k.M);
}
}  // namespace tobas::kdl
