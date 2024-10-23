#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/Matrix3d.hpp>
#include <tobas_kdl_msgs_adapter/Frame.hpp>
#include <tobas_kdl_msgs_adapter/Twist.hpp>
#include <tobas_kdl_msgs_adapter/Accel.hpp>

#include <tobas_msgs/msg/odometry.hpp>

namespace tobas_msgs
{
struct Odometry
{
  std_msgs::msg::Header header;
  kdl::Frame frame;
  kdl::Twist twist;
  kdl::Accel accel;
  Eigen::Matrix3d position_covariance;
  Eigen::Matrix3d orientation_covariance;
  Eigen::Matrix3d velocity_covariance;
  Eigen::Matrix3d gyro_covariance;
  Eigen::Matrix3d accel_covariance;
  Eigen::Matrix3d dgyro_covariance;
  int8_t status;

  using SharedPtr = std::shared_ptr<Odometry>;
  using ConstSharedPtr = std::shared_ptr<const Odometry>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::Odometry, tobas_msgs::msg::Odometry>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::Odometry;
  using ros_message_type = tobas_msgs::msg::Odometry;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameAdapter::convert_to_ros_message(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.position_covariance, dst.position_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.orientation_covariance, dst.orientation_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.velocity_covariance, dst.velocity_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.accel_covariance, dst.accel_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.dgyro_covariance, dst.dgyro_covariance);
    dst.status = src.status;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::FrameAdapter::convert_to_custom(src.frame, dst.frame);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, dst.twist);
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.accel, dst.accel);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.position_covariance, dst.position_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.orientation_covariance, dst.orientation_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.velocity_covariance, dst.velocity_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_covariance, dst.gyro_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.accel_covariance, dst.accel_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.dgyro_covariance, dst.dgyro_covariance);
    dst.status = src.status;
  }
};

namespace tobas_msgs
{
using OdometryAdapter = rclcpp::TypeAdapter<tobas_msgs::Odometry, tobas_msgs::msg::Odometry>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::Odometry, tobas_msgs::msg::Odometry);
