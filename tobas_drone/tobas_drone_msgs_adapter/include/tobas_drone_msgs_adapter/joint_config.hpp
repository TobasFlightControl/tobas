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
    dst.role = static_cast<uint8_t>(src.role);
    dst.cmd_iface = static_cast<uint8_t>(src.cmd_iface);
    dst.hw_iface = static_cast<uint8_t>(src.hw_iface);
    dst.home_pos = src.home_pos;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.name = src.name;
    dst.role = static_cast<tobas::jnt_role_t>(src.role);
    dst.cmd_iface = static_cast<tobas::jnt_cmd_iface_t>(src.cmd_iface);
    dst.hw_iface = static_cast<tobas::jnt_hw_iface_t>(src.hw_iface);
    dst.home_pos = src.home_pos;
  }
};

namespace tobas_drone_msgs
{
using JointConfigAdapter = rclcpp::TypeAdapter<tobas::JointConfig, tobas_drone_msgs::msg::JointConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::JointConfig, tobas_drone_msgs::msg::JointConfig);
