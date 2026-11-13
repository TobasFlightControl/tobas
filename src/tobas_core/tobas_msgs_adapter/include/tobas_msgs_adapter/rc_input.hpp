// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_constants/flight_mode.hpp>

#include <tobas_msgs/msg/rc_input.hpp>

namespace tobas_msgs
{
struct RCInput
{
  std_msgs::msg::Header header;
  uint8_t status;
  double roll;
  double pitch;
  double throttle;
  double yaw;
  tobas::FlightMode mode;
  bool sub_mode;
  bool enable;
  bool kill;
  std::array<bool, 8> gpsw;

  using SharedPtr = std::shared_ptr<RCInput>;
  using ConstSharedPtr = std::shared_ptr<const RCInput>;
  using UniquePtr = std::unique_ptr<RCInput>;
  using ConstUniquePtr = std::unique_ptr<const RCInput>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::RCInput, tobas_msgs::msg::RCInput>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::RCInput;
  using ros_message_type = tobas_msgs::msg::RCInput;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;
    _dst.status = _src.status;
    _dst.roll = _src.roll;
    _dst.pitch = _src.pitch;
    _dst.throttle = _src.throttle;
    _dst.yaw = _src.yaw;
    _dst.mode = static_cast<uint8_t>(_src.mode);
    _dst.sub_mode = _src.sub_mode;
    _dst.enable = _src.enable;
    _dst.kill = _src.kill;
    _dst.gpsw = _src.gpsw;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;
    _dst.status = _src.status;
    _dst.roll = _src.roll;
    _dst.pitch = _src.pitch;
    _dst.throttle = _src.throttle;
    _dst.yaw = _src.yaw;
    _dst.mode = static_cast<tobas::FlightMode>(_src.mode);
    _dst.sub_mode = _src.sub_mode;
    _dst.enable = _src.enable;
    _dst.kill = _src.kill;
    _dst.gpsw = _src.gpsw;
  }
};

namespace tobas_msgs
{
using RCInputAdapter = rclcpp::TypeAdapter<tobas_msgs::RCInput, tobas_msgs::msg::RCInput>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::RCInput, tobas_msgs::msg::RCInput);
