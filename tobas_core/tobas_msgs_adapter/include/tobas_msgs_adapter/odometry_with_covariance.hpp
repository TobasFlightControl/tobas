// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_msgs/msg/odometry_with_covariance.hpp>

#include "./odometry.hpp"

namespace tobas_msgs
{
struct OdometryWithCovariance
{
  Odometry odom;
  Eigen::Matrix3d position_covariance;
  Eigen::Matrix3d orientation_covariance;
  Eigen::Matrix3d velocity_covariance;
  Eigen::Matrix3d gyro_covariance;

  using SharedPtr = std::shared_ptr<OdometryWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const OdometryWithCovariance>;
  using UniquePtr = std::unique_ptr<OdometryWithCovariance>;
  using ConstUniquePtr = std::unique_ptr<const OdometryWithCovariance>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::OdometryWithCovariance, tobas_msgs::msg::OdometryWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::OdometryWithCovariance;
  using ros_message_type = tobas_msgs::msg::OdometryWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_msgs::OdometryAdapter::convert_to_ros_message(src.odom, dst.odom);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.position_covariance, dst.position_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.orientation_covariance, dst.orientation_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.velocity_covariance, dst.velocity_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_covariance, dst.gyro_covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_msgs::OdometryAdapter::convert_to_custom(src.odom, dst.odom);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.position_covariance, dst.position_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.orientation_covariance, dst.orientation_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.velocity_covariance, dst.velocity_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_covariance, dst.gyro_covariance);
  }
};

namespace tobas_msgs
{
using OdometryWithCovarianceAdapter =
  rclcpp::TypeAdapter<tobas_msgs::OdometryWithCovariance, tobas_msgs::msg::OdometryWithCovariance>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::OdometryWithCovariance, tobas_msgs::msg::OdometryWithCovariance);
