#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_std_tools/range.hpp>

#include <tobas_std_msgs/msg/range_float64.hpp>

namespace tobas_std_msgs
{
using RangeFloat64 = tobas_std::Range<double>;
using RangeFloat64Adapter = rclcpp::TypeAdapter<tobas_std_msgs::RangeFloat64, tobas_std_msgs::msg::RangeFloat64>;
}  // namespace tobas_std_msgs

template <>
struct rclcpp::TypeAdapter<tobas_std_msgs::RangeFloat64, tobas_std_msgs::msg::RangeFloat64>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_std_msgs::RangeFloat64;
  using ros_message_type = tobas_std_msgs::msg::RangeFloat64;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.lower = src.lower;
    dst.upper = src.upper;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.lower = src.lower;
    dst.upper = src.upper;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_std_msgs::RangeFloat64, tobas_std_msgs::msg::RangeFloat64);
