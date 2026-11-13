// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_rotor.hpp>

#include <tobas_drone_msgs/msg/electric_rotor_config.hpp>

#include "./rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::ElectricRotorConfig, tobas_drone_msgs::msg::ElectricRotorConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::ElectricRotorConfig;
  using ros_message_type = tobas_drone_msgs::msg::ElectricRotorConfig;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_ros_message(_src, _dst.rotor);

    _dst.channel = _src.channel;
    _dst.num_poles = _src.num_poles;
    _dst.kv = _src.kv;
    _dst.internal_resistance = _src.internal_resistance;
    _dst.min_speed = _src.min_speed;
    _dst.propeller_diameter = _src.propeller_diameter;
    _dst.motor_const = _src.motor_const;
    _dst.moment_const = _src.moment_const;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_custom(_src.rotor, _dst);

    _dst.channel = _src.channel;
    _dst.num_poles = _src.num_poles;
    _dst.kv = _src.kv;
    _dst.internal_resistance = _src.internal_resistance;
    _dst.min_speed = _src.min_speed;
    _dst.propeller_diameter = _src.propeller_diameter;
    _dst.motor_const = _src.motor_const;
    _dst.moment_const = _src.moment_const;
  }
};

namespace tobas_drone_msgs
{
using ElectricRotorConfigAdapter =
  rclcpp::TypeAdapter<tobas::ElectricRotorConfig, tobas_drone_msgs::msg::ElectricRotorConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::ElectricRotorConfig, tobas_drone_msgs::msg::ElectricRotorConfig);
