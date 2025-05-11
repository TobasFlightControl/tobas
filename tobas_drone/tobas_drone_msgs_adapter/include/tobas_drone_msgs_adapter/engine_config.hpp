#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/engine.hpp>

#include <tobas_drone_msgs/msg/engine_config.hpp>
#include <tobas_std_msgs_adapter/pair_float64.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::EngineConfig;
  using ros_message_type = tobas_drone_msgs::msg::EngineConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_std_msgs::PairFloat64Adapter::convert_to_ros_message(src.engine_const, dst.engine_const);
    dst.max_speed = src.max_speed;
    dst.hw_iface = static_cast<uint8_t>(src.hw_iface);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_std_msgs::PairFloat64Adapter::convert_to_custom(src.engine_const, dst.engine_const);
    dst.max_speed = src.max_speed;
    dst.hw_iface = static_cast<tobas::hw_iface_t>(src.hw_iface);
  }
};

namespace tobas_drone_msgs
{
using EngineConfigAdapter = rclcpp::TypeAdapter<tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::EngineConfig, tobas_drone_msgs::msg::EngineConfig);
