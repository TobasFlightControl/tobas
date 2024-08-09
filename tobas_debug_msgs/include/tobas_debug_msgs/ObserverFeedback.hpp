#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_eigen_msgs/Matrix3d.hpp>
#include <tobas_kdl_msgs/Vector.hpp>

#include <tobas_debug_msgs/msg/observer_feedback.hpp>

namespace tobas_debug_msgs
{
struct ObserverFeedback
{
  std_msgs::msg::Header header;
  kdl::Vector acc_bias;
  kdl::Vector gyro_bias;
  double gravity;
  Eigen::Matrix3d acc_bias_covariance;
  Eigen::Matrix3d gyro_bias_covariance;
  double gravity_variance;
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
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.acc_bias, dst.acc_bias);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.gyro_bias, dst.gyro_bias);
    dst.gravity = src.gravity;
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.acc_bias_covariance, dst.acc_bias_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_ros_message(src.gyro_bias_covariance, dst.gyro_bias_covariance);
    dst.gravity_variance = src.gravity_variance;
    dst.gps_anormaly_score = src.gps_anormaly_score;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.acc_bias, dst.acc_bias);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.gyro_bias, dst.gyro_bias);
    dst.gravity = src.gravity;
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.acc_bias_covariance, dst.acc_bias_covariance);
    tobas_eigen_msgs::Matrix3dAdapter::convert_to_custom(src.gyro_bias_covariance, dst.gyro_bias_covariance);
    dst.gravity_variance = src.gravity_variance;
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
