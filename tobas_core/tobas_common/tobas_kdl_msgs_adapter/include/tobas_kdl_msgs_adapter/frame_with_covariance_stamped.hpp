#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/msg/frame_with_covariance_stamped.hpp>

#include "./frame_with_covariance.hpp"

namespace tobas_kdl_msgs
{
struct FrameWithCovarianceStamped
{
  std_msgs::msg::Header header;
  tobas_kdl_msgs::FrameWithCovariance frame;

  using SharedPtr = std::shared_ptr<FrameWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const FrameWithCovarianceStamped>;
  using UniquePtr = std::unique_ptr<FrameWithCovarianceStamped>;
  using ConstUniquePtr = std::unique_ptr<const FrameWithCovarianceStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::FrameWithCovarianceStamped, tobas_kdl_msgs::msg::FrameWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::FrameWithCovarianceStamped;
  using ros_message_type = tobas_kdl_msgs::msg::FrameWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameWithCovarianceAdapter::convert_to_ros_message(src.frame, dst.frame);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameWithCovarianceAdapter::convert_to_custom(src.frame, dst.frame);
  }
};

namespace tobas_kdl_msgs
{
using FrameWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::FrameWithCovarianceStamped, tobas_kdl_msgs::msg::FrameWithCovarianceStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::FrameWithCovarianceStamped,
  tobas_kdl_msgs::msg::FrameWithCovarianceStamped);
