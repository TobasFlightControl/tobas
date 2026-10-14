// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl/quaternion.hpp>
#include <tobas_kdl/twist.hpp>
#include <tobas_kdl/vector.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace tobas
{
namespace kdl
{
void vectorKDLToMsg(const Vector& _k, geometry_msgs::msg::Vector3& _m);
void vectorMsgToKDL(const geometry_msgs::msg::Vector3& _m, Vector& _k);

void pointKDLToMsg(const Vector& _k, geometry_msgs::msg::Point& _m);
void pointMsgToKDL(const geometry_msgs::msg::Point& _m, Vector& _k);

void twistKDLToMsg(const Twist& _k, geometry_msgs::msg::Twist& _m);
void twistMsgToKDL(const geometry_msgs::msg::Twist& _m, Twist& _k);

void rotationKDLToMsg(const Rotation& _k, geometry_msgs::msg::Quaternion& _m);
void rotationMsgToKDL(const geometry_msgs::msg::Quaternion& _m, Rotation& _k);

void quaternionKDLToMsg(const Quaternion& _k, geometry_msgs::msg::Quaternion& _m);
void quaternionMsgToKDL(const geometry_msgs::msg::Quaternion& _m, Quaternion& _k);

void poseKDLToMsg(const Frame& _k, geometry_msgs::msg::Pose& _m);
void poseMsgToKDL(const geometry_msgs::msg::Pose& _m, Frame& _k);

void transformKDLToMsg(const Frame& _k, geometry_msgs::msg::Transform& _m);
void transformMsgToKDL(const geometry_msgs::msg::Transform& _m, Frame& _k);
}  // namespace kdl
}  // namespace tobas
