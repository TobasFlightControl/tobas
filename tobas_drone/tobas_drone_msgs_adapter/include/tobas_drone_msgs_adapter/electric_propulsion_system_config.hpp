#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>

#include <tobas_drone_msgs/msg/electric_propulsion_system_config.hpp>

#include "./battery_config.hpp"
#include "./electric_rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::ElectricPropulsionSystemConfig, tobas_drone_msgs::msg::ElectricPropulsionSystemConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::ElectricPropulsionSystemConfig;
  using ros_message_type = tobas_drone_msgs::msg::ElectricPropulsionSystemConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::BatteryConfigAdapter::convert_to_ros_message(src.battery, dst.battery);

    dst.rotors.clear();
    for (const auto& [_, src_rotor] : src.rotors) {
      const auto src_erotor = boost::polymorphic_pointer_downcast<tobas::ElectricRotorConfig>(src_rotor);
      dst.rotors.emplace_back();
      tobas_drone_msgs::ElectricRotorConfigAdapter::convert_to_ros_message(*src_erotor, dst.rotors.back());
    }
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::BatteryConfigAdapter::convert_to_custom(src.battery, dst.battery);

    dst.rotors.clear();
    for (const auto& src_erotor : src.rotors) {
      const auto dst_erotor = std::make_shared<tobas::ElectricRotorConfig>();
      tobas_drone_msgs::ElectricRotorConfigAdapter::convert_to_custom(src_erotor, *dst_erotor);
      dst.rotors[src_erotor.rotor.link_name] = dst_erotor;
    }
  }
};

namespace tobas_drone_msgs
{
using ElectricPropulsionSystemConfigAdapter =
  rclcpp::TypeAdapter<tobas::ElectricPropulsionSystemConfig, tobas_drone_msgs::msg::ElectricPropulsionSystemConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas::ElectricPropulsionSystemConfig,
  tobas_drone_msgs::msg::ElectricPropulsionSystemConfig);
