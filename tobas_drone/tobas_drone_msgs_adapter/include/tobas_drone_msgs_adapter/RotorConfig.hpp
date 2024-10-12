#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/rotor.hpp>
#include <tobas_drone_msgs/msg/rotor_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::RotorConfig;
  using ros_message_type = tobas_drone_msgs::msg::RotorConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.channel = src.channel;
    dst.link_name = src.link_name;
    dst.direction = static_cast<uint8_t>(src.direction);
    dst.axis = static_cast<uint8_t>(src.axis);
    dst.esc_mode = static_cast<uint8_t>(src.esc_mode);
    dst.num_poles = src.num_poles;
    dst.max_rot_speed = src.max_rot_speed;
    dst.motor_constant = src.motor_constant;
    dst.moment_constant = src.moment_constant;
    dst.drag_constant = src.drag_constant;
    dst.rot_speed_coefs.first = src.rot_speed_coefs.first;
    dst.rot_speed_coefs.second = src.rot_speed_coefs.second;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.channel = src.channel;
    dst.link_name = src.link_name;
    dst.direction = static_cast<tobas::turning_direction_t>(src.direction);
    dst.axis = static_cast<tobas::rotor_axis_t>(src.axis);
    dst.esc_mode = static_cast<tobas::esc_mode_t>(src.esc_mode);
    dst.num_poles = src.num_poles;
    dst.max_rot_speed = src.max_rot_speed;
    dst.motor_constant = src.motor_constant;
    dst.moment_constant = src.moment_constant;
    dst.drag_constant = src.drag_constant;
    dst.rot_speed_coefs.first = src.rot_speed_coefs.first;
    dst.rot_speed_coefs.second = src.rot_speed_coefs.second;
  }
};

namespace tobas_drone_msgs
{
using RotorConfigAdapter = rclcpp::TypeAdapter<tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::RotorConfig, tobas_drone_msgs::msg::RotorConfig);
