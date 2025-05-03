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

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_ros_message(src, dst.rotor);

    dst.channel = src.channel;
    dst.num_poles = src.num_poles;
    dst.kv = src.kv;
    dst.internal_resistance = src.internal_resistance;
    dst.propeller_diameter = src.propeller_diameter;
    dst.motor_const = src.motor_const;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_custom(src.rotor, dst);

    dst.channel = src.channel;
    dst.num_poles = src.num_poles;
    dst.kv = src.kv;
    dst.internal_resistance = src.internal_resistance;
    dst.propeller_diameter = src.propeller_diameter;
    dst.motor_const = src.motor_const;
  }
};

namespace tobas_drone_msgs
{
using ElectricRotorConfigAdapter =
  rclcpp::TypeAdapter<tobas::ElectricRotorConfig, tobas_drone_msgs::msg::ElectricRotorConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::ElectricRotorConfig, tobas_drone_msgs::msg::ElectricRotorConfig);
