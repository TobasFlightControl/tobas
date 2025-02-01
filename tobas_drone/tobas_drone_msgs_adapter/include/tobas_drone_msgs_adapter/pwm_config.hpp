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
    dst.joint_name = src.joint_name;
    dst.min_period = src.min_period;
    dst.max_period = src.max_period;
    dst.min_angle = src.min_angle;
    dst.max_angle = src.max_angle;
    dst.reverse = src.reverse;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.channel = src.channel;
    dst.joint_name = src.joint_name;
    dst.min_period = src.min_period;
    dst.max_period = src.max_period;
    dst.min_angle = src.min_angle;
    dst.max_angle = src.max_angle;
    dst.reverse = src.reverse;
  }
};

namespace tobas_drone_msgs
{
using PwmConfigAdapter = rclcpp::TypeAdapter<tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::PwmConfig, tobas_drone_msgs::msg::PwmConfig);
