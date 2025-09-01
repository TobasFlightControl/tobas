#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_debug_msgs/msg/multicopter_controller_feedback.hpp>

namespace tobas_debug_msgs
{
struct MulticopterControllerFeedback
{
  std_msgs::msg::Header header;
  kdl::Vector target_position;
  kdl::Vector target_velocity;
  kdl::Vector target_accel;
  kdl::Euler target_angle;
  kdl::Vector target_gyro;
  kdl::Vector target_dgyro;
  kdl::Vector position_integral_error;
  kdl::Vector angle_integral_error;

  using SharedPtr = std::shared_ptr<MulticopterControllerFeedback>;
  using ConstSharedPtr = std::shared_ptr<const MulticopterControllerFeedback>;
};
}  // namespace tobas_debug_msgs

template <>
struct rclcpp::
  TypeAdapter<tobas_debug_msgs::MulticopterControllerFeedback, tobas_debug_msgs::msg::MulticopterControllerFeedback>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_debug_msgs::MulticopterControllerFeedback;
  using ros_message_type = tobas_debug_msgs::msg::MulticopterControllerFeedback;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_position, dst.target_position);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_velocity, dst.target_velocity);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_accel, dst.target_accel);
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.target_angle, dst.target_angle);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_gyro, dst.target_gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_dgyro, dst.target_dgyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.position_integral_error, dst.position_integral_error);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.angle_integral_error, dst.angle_integral_error);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_position, dst.target_position);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_velocity, dst.target_velocity);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_accel, dst.target_accel);
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.target_angle, dst.target_angle);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_gyro, dst.target_gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_dgyro, dst.target_dgyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.position_integral_error, dst.position_integral_error);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.angle_integral_error, dst.angle_integral_error);
  }
};

namespace tobas_debug_msgs
{
using MultiRotorControllerFeedbackAdapter = rclcpp::
  TypeAdapter<tobas_debug_msgs::MulticopterControllerFeedback, tobas_debug_msgs::msg::MulticopterControllerFeedback>;
}  // namespace tobas_debug_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_debug_msgs::MulticopterControllerFeedback,
  tobas_debug_msgs::msg::MulticopterControllerFeedback);
