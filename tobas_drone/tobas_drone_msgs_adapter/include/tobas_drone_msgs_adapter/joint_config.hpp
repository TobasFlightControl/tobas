#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/joint/joint.hpp>
#include <tobas_drone_msgs/msg/joint_config.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::JointConfig, tobas_drone_msgs::msg::JointConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::JointConfig;
  using ros_message_type = tobas_drone_msgs::msg::JointConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.name = src.name;
    dst.home_pos = src.home_pos;
    dst.min_pos = src.min_pos;
    dst.max_pos = src.max_pos;
    dst.interface = static_cast<uint8_t>(src.interface);
    dst.role = static_cast<uint8_t>(src.role);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.name = src.name;
    dst.home_pos = src.home_pos;
    dst.min_pos = src.min_pos;
    dst.max_pos = src.max_pos;
    dst.interface = static_cast<tobas::joint_interface_t>(src.interface);
    dst.role = static_cast<tobas::joint_role_t>(src.role);
  }
};

namespace tobas_drone_msgs
{
using JointConfigAdapter = rclcpp::TypeAdapter<tobas::JointConfig, tobas_drone_msgs::msg::JointConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::JointConfig, tobas_drone_msgs::msg::JointConfig);
