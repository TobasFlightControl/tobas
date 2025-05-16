#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/magnetic_field_with_covariance_stamped.hpp>

#include "./magnetic_field_with_covariance.hpp"

namespace tobas_msgs
{
struct MagneticFieldWithCovarianceStamped
{
  std_msgs::msg::Header header;
  MagneticFieldWithCovariance mag;

  using SharedPtr = std::shared_ptr<MagneticFieldWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const MagneticFieldWithCovarianceStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::
  TypeAdapter<tobas_msgs::MagneticFieldWithCovarianceStamped, tobas_msgs::msg::MagneticFieldWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::MagneticFieldWithCovarianceStamped;
  using ros_message_type = tobas_msgs::msg::MagneticFieldWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::MagneticFieldWithCovarianceAdapter::convert_to_ros_message(src.mag, dst.mag);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::MagneticFieldWithCovarianceAdapter::convert_to_custom(src.mag, dst.mag);
  }
};

namespace tobas_msgs
{
using MagneticFieldWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_msgs::MagneticFieldWithCovarianceStamped, tobas_msgs::msg::MagneticFieldWithCovarianceStamped>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_msgs::MagneticFieldWithCovarianceStamped,
  tobas_msgs::msg::MagneticFieldWithCovarianceStamped);
