#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs_adapter/core.hpp>

#include <tobas_debug_msgs/msg/observer_feedback.hpp>

namespace tobas_debug_msgs
{
struct ObserverFeedback
{
  std_msgs::msg::Header header;

  Eigen::Vector3d position;
  Eigen::Vector3d velocity;
  Eigen::Vector4d hamilton;
  Eigen::Vector3d accel_bias;
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d mag_hard_bias;
  Eigen::Matrix3d mag_soft_bias;
  double gravity;

  Eigen::Matrix3d position_cov;
  Eigen::Matrix3d velocity_cov;
  Eigen::Matrix3d rotation_cov;
  Eigen::Matrix3d accel_bias_cov;
  Eigen::Matrix3d gyro_bias_cov;
  Eigen::Matrix3d mag_hard_bias_cov;
  Eigen::Matrix6d mag_soft_bias_cov;
  double gravity_var;

  double gps_anormaly_score;

  using SharedPtr = std::shared_ptr<ObserverFeedback>;
  using ConstSharedPtr = std::shared_ptr<const ObserverFeedback>;
};
}  // namespace tobas_debug_msgs

template <>
struct rclcpp::TypeAdapter<tobas_debug_msgs::ObserverFeedback, tobas_debug_msgs::msg::ObserverFeedback>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_debug_msgs::ObserverFeedback;
  using ros_message_type = tobas_debug_msgs::msg::ObserverFeedback;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;

    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(src.position, dst.position);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(src.velocity, dst.velocity);
    tobas_eigen_msgs::Vector4dAdapter::convert_to_ros_message(src.hamilton, dst.hamilton);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(src.accel_bias, dst.accel_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(src.gyro_bias, dst.gyro_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(src.mag_hard_bias, dst.mag_hard_bias);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.mag_soft_bias, dst.mag_soft_bias);
    dst.gravity = src.gravity;

    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.position_cov, dst.position_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.velocity_cov, dst.velocity_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.rotation_cov, dst.rotation_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.accel_bias_cov, dst.accel_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_bias_cov, dst.gyro_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.mag_hard_bias_cov, dst.mag_hard_bias_cov);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_ros_message(src.mag_soft_bias_cov, dst.mag_soft_bias_cov);
    dst.gravity_var = src.gravity_var;

    dst.gps_anormaly_score = src.gps_anormaly_score;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;

    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(src.position, dst.position);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(src.velocity, dst.velocity);
    tobas_eigen_msgs::Vector4dAdapter::convert_to_custom(src.hamilton, dst.hamilton);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(src.accel_bias, dst.accel_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(src.gyro_bias, dst.gyro_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(src.mag_hard_bias, dst.mag_hard_bias);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.mag_soft_bias, dst.mag_soft_bias);
    dst.gravity = src.gravity;

    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.position_cov, dst.position_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.velocity_cov, dst.velocity_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.rotation_cov, dst.rotation_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.accel_bias_cov, dst.accel_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_bias_cov, dst.gyro_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.mag_hard_bias_cov, dst.mag_hard_bias_cov);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_custom(src.mag_soft_bias_cov, dst.mag_soft_bias_cov);
    dst.gravity_var = src.gravity_var;

    dst.gps_anormaly_score = src.gps_anormaly_score;
  }
};

namespace tobas_debug_msgs
{
using ObserverFeedbackAdapter =
  rclcpp::TypeAdapter<tobas_debug_msgs::ObserverFeedback, tobas_debug_msgs::msg::ObserverFeedback>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_debug_msgs::ObserverFeedback,
  tobas_debug_msgs::msg::ObserverFeedback);
