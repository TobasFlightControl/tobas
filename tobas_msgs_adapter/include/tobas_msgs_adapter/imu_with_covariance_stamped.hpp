#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_msgs/msg/imu_with_covariance_stamped.hpp>

#include "./imu_with_covariance.hpp"

namespace tobas_msgs
{
struct ImuWithCovarianceStamped
{
  std_msgs::msg::Header header;
  ImuWithCovariance imu;

  using SharedPtr = std::shared_ptr<ImuWithCovarianceStamped>;
  using ConstSharedPtr = std::shared_ptr<const ImuWithCovarianceStamped>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::ImuWithCovarianceStamped, tobas_msgs::msg::ImuWithCovarianceStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::ImuWithCovarianceStamped;
  using ros_message_type = tobas_msgs::msg::ImuWithCovarianceStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::ImuWithCovarianceAdapter::convert_to_ros_message(src.imu, dst.imu);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_msgs::ImuWithCovarianceAdapter::convert_to_custom(src.imu, dst.imu);
  }
};

namespace tobas_msgs
{
using ImuWithCovarianceStampedAdapter =
  rclcpp::TypeAdapter<tobas_msgs::ImuWithCovarianceStamped, tobas_msgs::msg::ImuWithCovarianceStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_msgs::ImuWithCovarianceStamped,
  tobas_msgs::msg::ImuWithCovarianceStamped);
