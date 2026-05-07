// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/msg/twist_with_covariance_stamped.hpp>

#include "./twist_with_covariance.hpp"

namespace tobas_kdl_msgs
{
struct TwistWithCovarianceStamped
{
  std_msgs::msg::Header header;
  tobas_kdl_msgs::TwistWithCovariance twist;

  using SharedPtr = std::shared_ptr<TwistWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const TwistWithCovarianceStamped>;
  using UniquePtr = std::unique_ptr<TwistWithCovarianceStamped>;
  using ConstUniquePtr = std::unique_ptr<const TwistWithCovarianceStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::TwistWithCovarianceStamped, tobas_kdl_msgs::msg::TwistWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::TwistWithCovarianceStamped;
  using ros_message_type = tobas_kdl_msgs::msg::TwistWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::TwistWithCovarianceAdapter::convert_to_ros_message(src.twist, dst.twist);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::TwistWithCovarianceAdapter::convert_to_custom(src.twist, dst.twist);
  }
};

namespace tobas_kdl_msgs
{
using TwistWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::TwistWithCovarianceStamped, tobas_kdl_msgs::msg::TwistWithCovarianceStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::TwistWithCovarianceStamped,
  tobas_kdl_msgs::msg::TwistWithCovarianceStamped);
