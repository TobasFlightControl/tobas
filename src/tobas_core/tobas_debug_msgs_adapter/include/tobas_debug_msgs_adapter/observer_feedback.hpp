// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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
  double baro_alt_bias;
  double gravity;

  Eigen::Matrix3d position_cov;
  Eigen::Matrix3d velocity_cov;
  Eigen::Matrix3d rotation_cov;
  Eigen::Matrix3d accel_bias_cov;
  Eigen::Matrix3d gyro_bias_cov;
  Eigen::Matrix3d mag_hard_bias_cov;
  Eigen::Matrix<double, 6, 6> mag_soft_bias_cov;
  double baro_alt_bias_var;
  double gravity_var;

  double gnss_anomaly_score;

  using SharedPtr = std::shared_ptr<ObserverFeedback>;
  using ConstSharedPtr = std::shared_ptr<const ObserverFeedback>;
  using UniquePtr = std::unique_ptr<ObserverFeedback>;
  using ConstUniquePtr = std::unique_ptr<const ObserverFeedback>;
};
}  // namespace tobas_debug_msgs

template <>
struct rclcpp::TypeAdapter<tobas_debug_msgs::ObserverFeedback, tobas_debug_msgs::msg::ObserverFeedback>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_debug_msgs::ObserverFeedback;
  using ros_message_type = tobas_debug_msgs::msg::ObserverFeedback;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.header = _src.header;

    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(_src.position, _dst.position);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(_src.velocity, _dst.velocity);
    tobas_eigen_msgs::Vector4dAdapter::convert_to_ros_message(_src.hamilton, _dst.hamilton);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(_src.accel_bias, _dst.accel_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(_src.gyro_bias, _dst.gyro_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_ros_message(_src.mag_hard_bias, _dst.mag_hard_bias);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.mag_soft_bias, _dst.mag_soft_bias);
    _dst.baro_alt_bias = _src.baro_alt_bias;
    _dst.gravity = _src.gravity;

    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.position_cov, _dst.position_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.velocity_cov, _dst.velocity_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.rotation_cov, _dst.rotation_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.accel_bias_cov, _dst.accel_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.gyro_bias_cov, _dst.gyro_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(_src.mag_hard_bias_cov, _dst.mag_hard_bias_cov);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_ros_message(_src.mag_soft_bias_cov, _dst.mag_soft_bias_cov);
    _dst.baro_alt_bias_var = _src.baro_alt_bias_var;
    _dst.gravity_var = _src.gravity_var;

    _dst.gnss_anomaly_score = _src.gnss_anomaly_score;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.header = _src.header;

    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(_src.position, _dst.position);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(_src.velocity, _dst.velocity);
    tobas_eigen_msgs::Vector4dAdapter::convert_to_custom(_src.hamilton, _dst.hamilton);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(_src.accel_bias, _dst.accel_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(_src.gyro_bias, _dst.gyro_bias);
    tobas_eigen_msgs::Vector3dAdapter::convert_to_custom(_src.mag_hard_bias, _dst.mag_hard_bias);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.mag_soft_bias, _dst.mag_soft_bias);
    _dst.baro_alt_bias = _src.baro_alt_bias;
    _dst.gravity = _src.gravity;

    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.position_cov, _dst.position_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.velocity_cov, _dst.velocity_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.rotation_cov, _dst.rotation_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.accel_bias_cov, _dst.accel_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.gyro_bias_cov, _dst.gyro_bias_cov);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(_src.mag_hard_bias_cov, _dst.mag_hard_bias_cov);
    tobas_eigen_msgs::Matrix6dAdapter::convert_to_custom(_src.mag_soft_bias_cov, _dst.mag_soft_bias_cov);
    _dst.baro_alt_bias_var = _src.baro_alt_bias_var;
    _dst.gravity_var = _src.gravity_var;

    _dst.gnss_anomaly_score = _src.gnss_anomaly_score;
  }
};

namespace tobas_debug_msgs
{
using ObserverFeedbackAdapter =
  rclcpp::TypeAdapter<tobas_debug_msgs::ObserverFeedback, tobas_debug_msgs::msg::ObserverFeedback>;
}  // namespace tobas_debug_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_debug_msgs::ObserverFeedback, tobas_debug_msgs::msg::ObserverFeedback);
