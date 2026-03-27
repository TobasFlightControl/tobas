#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_tools/typedef.hpp>
#include <tobas_kdl/twist.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs/msg/twist_with_covariance.hpp>

#include "./twist.hpp"

namespace tobas_kdl_msgs
{
struct TwistWithCovariance
{
  tobas::kdl::Twist twist;
  Eigen::Matrix6d covariance;

  using SharedPtr = std::shared_ptr<TwistWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const TwistWithCovariance>;
  using UniquePtr = std::unique_ptr<TwistWithCovariance>;
  using ConstUniquePtr = std::unique_ptr<const TwistWithCovariance>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::TwistWithCovariance, tobas_kdl_msgs::msg::TwistWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::TwistWithCovariance;
  using ros_message_type = tobas_kdl_msgs::msg::TwistWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.twist, dst.twist);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_ros_message(src.covariance, dst.covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, dst.twist);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_custom(src.covariance, dst.covariance);
  }
};

namespace tobas_kdl_msgs
{
using TwistWithCovarianceAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::TwistWithCovariance, tobas_kdl_msgs::msg::TwistWithCovariance>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::TwistWithCovariance,
  tobas_kdl_msgs::msg::TwistWithCovariance);
