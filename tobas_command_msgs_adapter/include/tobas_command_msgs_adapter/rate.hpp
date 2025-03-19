#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/rate.hpp>

namespace tobas_command_msgs
{
struct Rate
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Vector rate;

  using SharedPtr = std::shared_ptr<Rate>;
  using ConstSharedPtr = std::shared_ptr<const Rate>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::Rate, tobas_command_msgs::msg::Rate>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::Rate;
  using ros_message_type = tobas_command_msgs::msg::Rate;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.rate, dst.rate);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.rate, dst.rate);
  }
};

namespace tobas_command_msgs
{
using RateAdapter = rclcpp::TypeAdapter<tobas_command_msgs::Rate, tobas_command_msgs::msg::Rate>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::Rate, tobas_command_msgs::msg::Rate);
