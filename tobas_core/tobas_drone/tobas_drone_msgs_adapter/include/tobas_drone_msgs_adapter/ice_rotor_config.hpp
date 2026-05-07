// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_rotor.hpp>

#include <tobas_drone_msgs/msg/ice_rotor_config.hpp>
#include <tobas_std_msgs_adapter/range_float64.hpp>

#include <tobas_drone_msgs_adapter/vpp_moment_constant.hpp>
#include <tobas_drone_msgs_adapter/vpp_motor_constant.hpp>

#include "./rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::IceRotorConfig, tobas_drone_msgs::msg::IceRotorConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::IceRotorConfig;
  using ros_message_type = tobas_drone_msgs::msg::IceRotorConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_ros_message(src, dst.rotor);

    dst.gear_ratio = src.gear_ratio;
    tobas_std_msgs::RangeFloat64Adapter::convert_to_ros_message(src.pitch_limit, dst.pitch_limit);
    dst.center_pitch = src.center_pitch;
    tobas_drone_msgs::VppMotorConstantAdapter::convert_to_ros_message(src.motor_const, dst.motor_const);
    tobas_drone_msgs::VppMomentConstantAdapter::convert_to_ros_message(src.moment_const, dst.moment_const);
    dst.hw_iface = static_cast<uint8_t>(src.hw_iface);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_custom(src.rotor, dst);

    dst.gear_ratio = src.gear_ratio;
    tobas_std_msgs::RangeFloat64Adapter::convert_to_custom(src.pitch_limit, dst.pitch_limit);
    dst.center_pitch = src.center_pitch;
    tobas_drone_msgs::VppMotorConstantAdapter::convert_to_custom(src.motor_const, dst.motor_const);
    tobas_drone_msgs::VppMomentConstantAdapter::convert_to_custom(src.moment_const, dst.moment_const);
    dst.hw_iface = static_cast<tobas::HardwareInterface>(src.hw_iface);
  }
};

namespace tobas_drone_msgs
{
using IceRotorConfigAdapter = rclcpp::TypeAdapter<tobas::IceRotorConfig, tobas_drone_msgs::msg::IceRotorConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::IceRotorConfig, tobas_drone_msgs::msg::IceRotorConfig);
