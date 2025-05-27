#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_rotor.hpp>

#include <tobas_drone_msgs/msg/ice_rotor_config.hpp>
#include <tobas_std_msgs_adapter/pair_float64.hpp>
#include <tobas_std_msgs_adapter/range_float64.hpp>

#include "./rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::ICERotorConfig, tobas_drone_msgs::msg::ICERotorConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::ICERotorConfig;
  using ros_message_type = tobas_drone_msgs::msg::ICERotorConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_ros_message(src, dst.rotor);

    dst.gear_ratio = src.gear_ratio;
    dst.pitch_ref = src.pitch_ref;
    tobas_std_msgs::RangeFloat64Adapter::convert_to_ros_message(src.pitch_limit, dst.pitch_limit);
    tobas_std_msgs::PairFloat64Adapter::convert_to_ros_message(src.motor_const, dst.motor_const);
    dst.hw_iface = static_cast<uint8_t>(src.hw_iface);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::RotorConfigAdapter::convert_to_custom(src.rotor, dst);

    dst.gear_ratio = src.gear_ratio;
    dst.pitch_ref = src.pitch_ref;
    tobas_std_msgs::RangeFloat64Adapter::convert_to_custom(src.pitch_limit, dst.pitch_limit);
    tobas_std_msgs::PairFloat64Adapter::convert_to_custom(src.motor_const, dst.motor_const);
    dst.hw_iface = static_cast<tobas::hw_iface_t>(src.hw_iface);
  }
};

namespace tobas_drone_msgs
{
using ICERotorConfigAdapter = rclcpp::TypeAdapter<tobas::ICERotorConfig, tobas_drone_msgs::msg::ICERotorConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::ICERotorConfig, tobas_drone_msgs::msg::ICERotorConfig);
