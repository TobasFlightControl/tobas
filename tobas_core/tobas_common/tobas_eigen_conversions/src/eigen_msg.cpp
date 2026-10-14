// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_conversions/eigen_msg.hpp"

namespace tobas::tf
{
void vectorEigenToMsg(const Eigen::Vector3d& _e, geometry_msgs::msg::Vector3& _m)
{
  _m.x = _e.x();
  _m.y = _e.y();
  _m.z = _e.z();
}

void vectorMsgToEigen(const geometry_msgs::msg::Vector3& _m, Eigen::Vector3d& _e)
{
  _e.x() = _m.x;
  _e.y() = _m.y;
  _e.z() = _m.z;
}

void pointEigenToMsg(const Eigen::Vector3d& _e, geometry_msgs::msg::Point& _m)
{
  _m.x = _e.x();
  _m.y = _e.y();
  _m.z = _e.z();
}

void pointMsgToEigen(const geometry_msgs::msg::Point& _m, Eigen::Vector3d& _e)
{
  _e.x() = _m.x;
  _e.y() = _m.y;
  _e.z() = _m.z;
}

void point32EigenToMsg(const Eigen::Vector3f& _e, geometry_msgs::msg::Point32& _m)
{
  _m.x = _e.x();
  _m.y = _e.y();
  _m.z = _e.z();
}

void point32MsgToEigen(const geometry_msgs::msg::Point32& _m, Eigen::Vector3f& _e)
{
  _e.x() = _m.x;
  _e.y() = _m.y;
  _e.z() = _m.z;
}

void quaternionEigenToMsg(const Eigen::Quaterniond& _e, geometry_msgs::msg::Quaternion& _m)
{
  _m.w = _e.w();
  _m.x = _e.x();
  _m.y = _e.y();
  _m.z = _e.z();
}

void quaternionMsgToEigen(const geometry_msgs::msg::Quaternion& _m, Eigen::Quaterniond& _e)
{
  _e.w() = _m.w;
  _e.x() = _m.x;
  _e.y() = _m.y;
  _e.z() = _m.z;
}

void matrix3EigenToMsg(const Eigen::Matrix3d& _e, std::array<double, 9>& _m)
{
  std::memcpy(_m.data(), _e.data(), sizeof(double) * 9);
}

void matrix3MsgToEigen(const std::array<double, 9>& _m, Eigen::Matrix3d& _e)
{
  std::memcpy(_e.data(), _m.data(), sizeof(double) * 9);
}
}  // namespace tobas::tf
