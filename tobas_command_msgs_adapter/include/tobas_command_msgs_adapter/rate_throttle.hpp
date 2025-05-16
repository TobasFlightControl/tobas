#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/rate_throttle.hpp>

namespace tobas_command_msgs
{
struct RateThrottle
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Vector rate;
  double throttle;

  using SharedPtr = std::shared_ptr<RateThrottle>;
  using ConstSharedPtr = std::shared_ptr<const RateThrottle>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::RateThrottle, tobas_command_msgs::msg::RateThrottle>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::RateThrottle;
  using ros_message_type = tobas_command_msgs::msg::RateThrottle;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.rate, dst.rate);
    dst.throttle = src.throttle;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.rate, dst.rate);
    dst.throttle = src.throttle;
  }
};

namespace tobas_command_msgs
{
using RateThrottleAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::RateThrottle, tobas_command_msgs::msg::RateThrottle>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::RateThrottle, tobas_command_msgs::msg::RateThrottle);
