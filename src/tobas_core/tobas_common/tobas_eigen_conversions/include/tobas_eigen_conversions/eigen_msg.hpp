// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace tobas::tf
{
void vectorEigenToMsg(const Eigen::Vector3d& _e, geometry_msgs::msg::Vector3& _m);
void vectorMsgToEigen(const geometry_msgs::msg::Vector3& _m, Eigen::Vector3d& _e);

void pointEigenToMsg(const Eigen::Vector3d& _e, geometry_msgs::msg::Point& _m);
void pointMsgToEigen(const geometry_msgs::msg::Point& _m, Eigen::Vector3d& _e);

void point32EigenToMsg(const Eigen::Vector3f& _e, geometry_msgs::msg::Point32& _m);
void point32MsgToEigen(const geometry_msgs::msg::Point32& _m, Eigen::Vector3f& _e);

void quaternionEigenToMsg(const Eigen::Quaterniond& _e, geometry_msgs::msg::Quaternion& _m);
void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion& _m, Eigen::Quaterniond& _e);

void matrix3EigenToMsg(const Eigen::Matrix3d& _e, std::array<double, 9>& _m);
void matrix3MsgToEigen(const std::array<double, 9>& _m, Eigen::Matrix3d& _e);
}  // namespace tobas::tf
