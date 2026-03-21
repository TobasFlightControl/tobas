#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/msg/accel_with_covariance_stamped.hpp>

#include "./accel_with_covariance.hpp"

namespace tobas_kdl_msgs
{
struct AccelWithCovarianceStamped
{
  std_msgs::msg::Header header;
  tobas_kdl_msgs::AccelWithCovariance accel;

  using SharedPtr = std::shared_ptr<AccelWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const AccelWithCovarianceStamped>;
  using UniquePtr = std::unique_ptr<AccelWithCovarianceStamped>;
  using ConstUniquePtr = std::unique_ptr<const AccelWithCovarianceStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::AccelWithCovarianceStamped, tobas_kdl_msgs::msg::AccelWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::AccelWithCovarianceStamped;
  using ros_message_type = tobas_kdl_msgs::msg::AccelWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::AccelWithCovarianceAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::AccelWithCovarianceAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_kdl_msgs
{
using AccelWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::AccelWithCovarianceStamped, tobas_kdl_msgs::msg::AccelWithCovarianceStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::AccelWithCovarianceStamped,
  tobas_kdl_msgs::msg::AccelWithCovarianceStamped);
