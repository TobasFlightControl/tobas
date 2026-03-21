#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>

#include "./odometry_with_covariance.hpp"

namespace tobas_msgs
{
struct OdometryWithCovarianceStamped
{
  std_msgs::msg::Header header;
  OdometryWithCovariance odom;

  using SharedPtr = std::shared_ptr<OdometryWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const OdometryWithCovarianceStamped>;
  using UniquePtr = std::unique_ptr<OdometryWithCovarianceStamped>;
  using ConstUniquePtr = std::unique_ptr<const OdometryWithCovarianceStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::OdometryWithCovarianceStamped, tobas_msgs::msg::OdometryWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::OdometryWithCovarianceStamped;
  using ros_message_type = tobas_msgs::msg::OdometryWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::OdometryWithCovarianceAdapter::convert_to_ros_message(src.odom, dst.odom);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::OdometryWithCovarianceAdapter::convert_to_custom(src.odom, dst.odom);
  }
};

namespace tobas_msgs
{
using OdometryWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_msgs::OdometryWithCovarianceStamped, tobas_msgs::msg::OdometryWithCovarianceStamped>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_msgs::OdometryWithCovarianceStamped,
  tobas_msgs::msg::OdometryWithCovarianceStamped);
