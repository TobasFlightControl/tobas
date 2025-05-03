#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/frame_vel.hpp>
#include <tobas_kdl_msgs/msg/frame_vel_stamped.hpp>

#include "./frame_vel.hpp"

namespace tobas_kdl_msgs
{
struct FrameVelStamped
{
  std_msgs::msg::Header header;
  kdl::FrameVel frame_vel;

  using SharedPtr = std::shared_ptr<FrameVelStamped>;
  using ConstSharedPtr = std::shared_ptr<const FrameVelStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::FrameVelStamped, tobas_kdl_msgs::msg::FrameVelStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::FrameVelStamped;
  using ros_message_type = tobas_kdl_msgs::msg::FrameVelStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameVelAdapter::convert_to_ros_message(src.frame_vel, dst.frame_vel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameVelAdapter::convert_to_custom(src.frame_vel, dst.frame_vel);
  }
};

namespace tobas_kdl_msgs
{
using FrameVelStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::FrameVelStamped, tobas_kdl_msgs::msg::FrameVelStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::FrameVelStamped, tobas_kdl_msgs::msg::FrameVelStamped);
