#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/rate_throttle_vector.hpp>

namespace tobas_command_msgs
{
struct RateThrottleVector
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::Priority priority;
  kdl::Vector rate;
  double throttle;
  double thrust_angle;

  using SharedPtr = std::shared_ptr<RateThrottleVector>;
  using ConstSharedPtr = std::shared_ptr<const RateThrottleVector>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::RateThrottleVector, tobas_command_msgs::msg::RateThrottleVector>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::RateThrottleVector;
  using ros_message_type = tobas_command_msgs::msg::RateThrottleVector;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.rate, dst.rate);
    dst.throttle = src.throttle;
    dst.thrust_angle = src.thrust_angle;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.priority = src.priority;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.rate, dst.rate);
    dst.throttle = src.throttle;
    dst.thrust_angle = src.thrust_angle;
  }
};

namespace tobas_command_msgs
{
using RateThrottleVectorAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::RateThrottleVector, tobas_command_msgs::msg::RateThrottleVector>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_command_msgs::RateThrottleVector,
  tobas_command_msgs::msg::RateThrottleVector);
