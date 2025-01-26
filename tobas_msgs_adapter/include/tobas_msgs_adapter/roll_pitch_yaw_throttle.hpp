#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_msgs/msg/roll_pitch_yaw_throttle.hpp>

namespace tobas_msgs
{
struct RollPitchYawThrottle
{
  std_msgs::msg::Header header;
  tobas_msgs::msg::CommandLevel level;
  kdl::Euler rpy;
  double throttle;

  using SharedPtr = std::shared_ptr<RollPitchYawThrottle>;
  using ConstSharedPtr = std::shared_ptr<const RollPitchYawThrottle>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::RollPitchYawThrottle, tobas_msgs::msg::RollPitchYawThrottle>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::RollPitchYawThrottle;
  using ros_message_type = tobas_msgs::msg::RollPitchYawThrottle;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.rpy, dst.rpy);
    dst.throttle = src.throttle;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.rpy, dst.rpy);
    dst.throttle = src.throttle;
  }
};

namespace tobas_msgs
{
using RollPitchYawThrottleAdapter =
  rclcpp::TypeAdapter<tobas_msgs::RollPitchYawThrottle, tobas_msgs::msg::RollPitchYawThrottle>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::RollPitchYawThrottle, tobas_msgs::msg::RollPitchYawThrottle);
