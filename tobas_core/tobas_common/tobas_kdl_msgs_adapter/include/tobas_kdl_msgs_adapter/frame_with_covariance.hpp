#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/frame.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs/msg/frame_with_covariance.hpp>

#include "./frame.hpp"

namespace tobas_kdl_msgs
{
struct FrameWithCovariance
{
  tobas::kdl::Frame frame;
  Eigen::Matrix6d covariance;

  using SharedPtr = std::shared_ptr<FrameWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const FrameWithCovariance>;
  using UniquePtr = std::unique_ptr<FrameWithCovariance>;
  using ConstUniquePtr = std::unique_ptr<const FrameWithCovariance>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::FrameWithCovariance, tobas_kdl_msgs::msg::FrameWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::FrameWithCovariance;
  using ros_message_type = tobas_kdl_msgs::msg::FrameWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame, dst.frame);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_ros_message(src.covariance, dst.covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, dst.frame);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_custom(src.covariance, dst.covariance);
  }
};

namespace tobas_kdl_msgs
{
using FrameWithCovarianceAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::FrameWithCovariance, tobas_kdl_msgs::msg::FrameWithCovariance>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::FrameWithCovariance,
  tobas_kdl_msgs::msg::FrameWithCovariance);
