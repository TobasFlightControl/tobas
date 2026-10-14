// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>

#include <tobas_drone_msgs/msg/ice_propulsion_system_config.hpp>

#include "./engine_config.hpp"
#include "./ice_rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::IcePropulsionSystemConfig, tobas_drone_msgs::msg::IcePropulsionSystemConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::IcePropulsionSystemConfig;
  using ros_message_type = tobas_drone_msgs::msg::IcePropulsionSystemConfig;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_drone_msgs::EngineConfigAdapter::convert_to_ros_message(_src.engine, _dst.engine);

    _dst.rotors.clear();
    for (const auto& [_, src_rotor] : _src.rotors) {
      const auto src_irotor = std::static_pointer_cast<tobas::IceRotorConfig>(src_rotor);
      _dst.rotors.emplace_back();
      tobas_drone_msgs::IceRotorConfigAdapter::convert_to_ros_message(*src_irotor, _dst.rotors.back());
    }
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_drone_msgs::EngineConfigAdapter::convert_to_custom(_src.engine, _dst.engine);

    _dst.rotors.clear();
    for (const auto& src_irotor : _src.rotors) {
      const auto dst_irotor = std::make_shared<tobas::IceRotorConfig>();
      tobas_drone_msgs::IceRotorConfigAdapter::convert_to_custom(src_irotor, *dst_irotor);
      _dst.rotors[src_irotor.rotor.link_name] = dst_irotor;
    }
  }
};

namespace tobas_drone_msgs
{
using IcePropulsionSystemConfigAdapter =
  rclcpp::TypeAdapter<tobas::IcePropulsionSystemConfig, tobas_drone_msgs::msg::IcePropulsionSystemConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas::IcePropulsionSystemConfig,
  tobas_drone_msgs::msg::IcePropulsionSystemConfig);
