#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/euler.hpp>
#include <tobas_kdl_msgs/msg/euler.hpp>

template <>
struct rclcpp::TypeAdapter<kdl::Euler, tobas_kdl_msgs::msg::Euler>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Euler;
  using ros_message_type = tobas_kdl_msgs::msg::Euler;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.roll = src.roll;
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.roll = src.roll;
    dst.pitch = src.pitch;
    dst.yaw = src.yaw;
  }
};

namespace tobas_kdl_msgs
{
using EulerAdapter = rclcpp::TypeAdapter<kdl::Euler, tobas_kdl_msgs::msg::Euler>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Euler, tobas_kdl_msgs::msg::Euler);
