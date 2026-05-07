// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/odometry_stamped.hpp>

#include "./odometry.hpp"

namespace tobas_msgs
{
struct OdometryStamped
{
  std_msgs::msg::Header header;
  Odometry odom;

  using SharedPtr = std::shared_ptr<OdometryStamped>;
  using ConstSharedPtr = std::shared_ptr<const OdometryStamped>;
  using UniquePtr = std::unique_ptr<OdometryStamped>;
  using ConstUniquePtr = std::unique_ptr<const OdometryStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::OdometryStamped, tobas_msgs::msg::OdometryStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::OdometryStamped;
  using ros_message_type = tobas_msgs::msg::OdometryStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::OdometryAdapter::convert_to_ros_message(src.odom, dst.odom);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::OdometryAdapter::convert_to_custom(src.odom, dst.odom);
  }
};

namespace tobas_msgs
{
using OdometryStampedAdapter = rclcpp::TypeAdapter<tobas_msgs::OdometryStamped, tobas_msgs::msg::OdometryStamped>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::OdometryStamped, tobas_msgs::msg::OdometryStamped);
