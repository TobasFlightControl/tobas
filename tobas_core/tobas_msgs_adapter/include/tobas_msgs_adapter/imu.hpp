// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_msgs/msg/imu.hpp>

#include "./imu.hpp"

namespace tobas_msgs
{
struct Imu
{
  std_msgs::msg::Header header;
  tobas::kdl::Vector accel;
  tobas::kdl::Vector gyro;
  tobas::kdl::Vector dgyro;

  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<const Imu>;
  using UniquePtr = std::unique_ptr<Imu>;
  using ConstUniquePtr = std::unique_ptr<const Imu>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Imu;
  using ros_message_type = tobas_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.accel, _dst.accel);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.gyro, _dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.dgyro, _dst.dgyro);
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.accel, _dst.accel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.gyro, _dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.dgyro, _dst.dgyro);
  }
};

namespace tobas_msgs
{
using ImuAdapter = rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Imu, tobas_msgs::msg::Imu);
