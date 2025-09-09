#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/frame.hpp>

#include <tobas_kdl_msgs/msg/frame_stamped.hpp>

#include "./frame.hpp"

namespace tobas_kdl_msgs
{
struct FrameStamped
{
  std_msgs::msg::Header header;
  kdl::Frame frame;

  using SharedPtr = std::shared_ptr<FrameStamped>;
  using ConstSharedPtr = std::shared_ptr<const FrameStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::FrameStamped, tobas_kdl_msgs::msg::FrameStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::FrameStamped;
  using ros_message_type = tobas_kdl_msgs::msg::FrameStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame, dst.frame);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, dst.frame);
  }
};

namespace tobas_kdl_msgs
{
using FrameStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::FrameStamped, tobas_kdl_msgs::msg::FrameStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::FrameStamped, tobas_kdl_msgs::msg::FrameStamped);
