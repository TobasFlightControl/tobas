#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_std_msgs/msg/pair_float64.hpp>

namespace tobas_std_msgs
{
using PairFloat64 = std::pair<double, double>;
using PairFloat64Adapter = rclcpp::TypeAdapter<tobas_std_msgs::PairFloat64, tobas_std_msgs::msg::PairFloat64>;
}  // namespace tobas_std_msgs

template <>
struct rclcpp::TypeAdapter<tobas_std_msgs::PairFloat64, tobas_std_msgs::msg::PairFloat64>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_std_msgs::PairFloat64;
  using ros_message_type = tobas_std_msgs::msg::PairFloat64;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.first = src.first;
    dst.second = src.second;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.first = src.first;
    dst.second = src.second;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_std_msgs::PairFloat64, tobas_std_msgs::msg::PairFloat64);
