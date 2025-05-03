#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/pwm.hpp>
#include <tobas_drone_msgs/msg/pwm_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::PwmConfig;
  using ros_message_type = tobas_drone_msgs::msg::PwmConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.channel = src.channel;
    dst.name = src.name;
    dst.min_period = src.period_range.lower;
    dst.max_period = src.period_range.upper;
    dst.min_value = src.value_range.lower;
    dst.max_value = src.value_range.upper;
    dst.reverse = src.reverse;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.channel = src.channel;
    dst.name = src.name;
    dst.period_range.lower = src.min_period;
    dst.period_range.upper = src.max_period;
    dst.value_range.lower = src.min_value;
    dst.value_range.upper = src.max_value;
    dst.reverse = src.reverse;
  }
};

namespace tobas_drone_msgs
{
using PwmConfigAdapter = rclcpp::TypeAdapter<tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig);
