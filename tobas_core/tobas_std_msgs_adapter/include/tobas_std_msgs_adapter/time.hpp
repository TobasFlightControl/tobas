#pragma once

#include <rclcpp/type_adapter.hpp>
#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>

template <>
struct rclcpp::TypeAdapter<rclcpp::Time, builtin_interfaces::msg::Time>
{
  using is_specialized = std::true_type;
  using custom_type = rclcpp::Time;
  using ros_message_type = builtin_interfaces::msg::Time;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    const auto ns = src.nanoseconds();
    assert(ns >= 0);
    dst.sec = ns / 1'000'000'000;
    dst.nanosec = ns % 1'000'000'000;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    const auto ns = RCL_S_TO_NS(src.sec) + src.nanosec;
    dst = custom_type(ns);
  }
};

namespace tobas_std_msgs
{
using TimeAdapter = rclcpp::TypeAdapter<rclcpp::Time, builtin_interfaces::msg::Time>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(rclcpp::Time, builtin_interfaces::msg::Time);
