#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_msgs/msg/magnetic_field_with_covariance.hpp>

namespace tobas_msgs
{
struct MagneticFieldWithCovariance
{
  kdl::Vector mag;
  Eigen::Matrix3d covariance;

  using SharedPtr = std::shared_ptr<MagneticFieldWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const MagneticFieldWithCovariance>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::MagneticFieldWithCovariance, tobas_msgs::msg::MagneticFieldWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::MagneticFieldWithCovariance;
  using ros_message_type = tobas_msgs::msg::MagneticFieldWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.mag, dst.mag);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.covariance, dst.covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.mag, dst.mag);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.covariance, dst.covariance);
  }
};

namespace tobas_msgs
{
using MagneticFieldWithCovarianceAdapter =
  rclcpp::TypeAdapter<tobas_msgs::MagneticFieldWithCovariance, tobas_msgs::msg::MagneticFieldWithCovariance>;
}  // namespace tobas_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_msgs::MagneticFieldWithCovariance,
  tobas_msgs::msg::MagneticFieldWithCovariance);
