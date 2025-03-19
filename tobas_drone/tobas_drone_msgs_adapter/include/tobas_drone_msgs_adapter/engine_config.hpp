#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp>
#include <tobas_drone_msgs/msg/engine_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::EngineConfig;
  using ros_message_type = tobas_drone_msgs::msg::EngineConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.torque_const = src.torque_const;
    dst.friction_torque = src.friction_torque;
    dst.hw_iface = static_cast<uint8_t>(src.hw_iface);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.torque_const = src.torque_const;
    dst.friction_torque = src.friction_torque;
    dst.hw_iface = static_cast<tobas::hw_iface_t>(src.hw_iface);
  }
};

namespace tobas_drone_msgs
{
using EngineConfigAdapter = rclcpp::TypeAdapter<tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig);
