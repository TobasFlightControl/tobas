#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/electric_propulsion_system/battery.hpp>
#include <tobas_drone_msgs/msg/battery_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::BatteryConfig, tobas_drone_msgs::msg::BatteryConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::BatteryConfig;
  using ros_message_type = tobas_drone_msgs::msg::BatteryConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.nominal_voltage = src.nominal_voltage;
    dst.max_voltage = src.max_voltage;
    dst.sag_voltage = src.sag_voltage;
    dst.max_current = src.max_current;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.nominal_voltage = src.nominal_voltage;
    dst.max_voltage = src.max_voltage;
    dst.sag_voltage = src.sag_voltage;
    dst.max_current = src.max_current;
  }
};

namespace tobas_drone_msgs
{
using BatteryConfigAdapter = rclcpp::TypeAdapter<tobas::BatteryConfig, tobas_drone_msgs::msg::BatteryConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::BatteryConfig, tobas_drone_msgs::msg::BatteryConfig);
