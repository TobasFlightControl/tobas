#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/Matrix3d.hpp>
#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_msgs/msg/imu_with_covariance.hpp>

#include "./Imu.hpp"

namespace tobas_msgs
{
struct ImuWithCovariance
{
  Imu imu;
  Eigen::Matrix3d gyro_covariance;
  Eigen::Matrix3d accel_covariance;

  using SharedPtr = std::shared_ptr<ImuWithCovariance>;
  using ConstSharedPtr = std::shared_ptr<const ImuWithCovariance>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::ImuWithCovariance, tobas_msgs::msg::ImuWithCovariance>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::ImuWithCovariance;
  using ros_message_type = tobas_msgs::msg::ImuWithCovariance;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_msgs::ImuAdapter::convert_to_ros_message(src.imu, dst.imu);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.accel_covariance, dst.accel_covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_msgs::ImuAdapter::convert_to_custom(src.imu, dst.imu);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.accel_covariance, dst.accel_covariance);
  }
};

namespace tobas_msgs
{
using ImuWithCovarianceAdapter = rclcpp::TypeAdapter<tobas_msgs::ImuWithCovariance, tobas_msgs::msg::ImuWithCovariance>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::ImuWithCovariance, tobas_msgs::msg::ImuWithCovariance);
