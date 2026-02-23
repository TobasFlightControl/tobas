#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/accel.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs/msg/accel_with_covariance.hpp>

#include "./accel.hpp"

namespace tobas_kdl_msgs
{
struct AccelWithCovariance
{
  kdl::Accel accel;
  Eigen::Matrix6d covariance;

  using SharedPtr = std::shared_ptr<AccelWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const AccelWithCovariance>;
  using UniquePtr = std::unique_ptr<AccelWithCovariance>;
  using ConstUniquePtr = std::unique_ptr<const AccelWithCovariance>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::AccelWithCovariance, tobas_kdl_msgs::msg::AccelWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::AccelWithCovariance;
  using ros_message_type = tobas_kdl_msgs::msg::AccelWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_ros_message(src.covariance, dst.covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_custom(src.covariance, dst.covariance);
  }
};

namespace tobas_kdl_msgs
{
using AccelWithCovarianceAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::AccelWithCovariance, tobas_kdl_msgs::msg::AccelWithCovariance>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::AccelWithCovariance,
  tobas_kdl_msgs::msg::AccelWithCovariance);
