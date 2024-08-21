#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/Matrix3d.hpp>
#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_msgs/msg/imu.hpp>

namespace tobas_msgs
{
struct Imu
{
  std_msgs::msg::Header header;
  kdl::Vector gyro;
  kdl::Vector accel;
  Eigen::Matrix3d gyro_covariance;
  Eigen::Matrix3d accel_covariance;

  using SharedPtr = std::shared_ptr<Imu>;
  using ConstSharedPtr = std::shared_ptr<const Imu>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Imu;
  using ros_message_type = tobas_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.accel_covariance, dst.accel_covariance);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.accel_covariance, dst.accel_covariance);
  }
};

namespace tobas_msgs
{
using ImuAdapter = rclcpp::TypeAdapter<tobas_msgs::Imu, tobas_msgs::msg::Imu>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Imu, tobas_msgs::msg::Imu);
