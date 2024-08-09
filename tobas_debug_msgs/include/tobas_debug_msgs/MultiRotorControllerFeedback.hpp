#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/Vector.hpp>
#include <tobas_kdl_msgs/Euler.hpp>

#include <tobas_debug_msgs/msg/multi_rotor_controller_feedback.hpp>

namespace tobas_debug_msgs
{
struct MultiRotorControllerFeedback
{
  std_msgs::msg::Header header;
  kdl::Vector target_position;
  kdl::Euler target_orientation;
  kdl::Vector target_velocity_global;
  kdl::Vector target_velocity_local;
  kdl::Vector target_accel_global;
  kdl::Vector target_accel_local;
  double target_thrust;
  kdl::Vector position_integral_error;

  using SharedPtr = std::shared_ptr<MultiRotorControllerFeedback>;
  using ConstSharedPtr = std::shared_ptr<const MultiRotorControllerFeedback>;
};
}  // namespace tobas_debug_msgs

template <>
struct rclcpp::
  TypeAdapter<tobas_debug_msgs::MultiRotorControllerFeedback, tobas_debug_msgs::msg::MultiRotorControllerFeedback>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_debug_msgs::MultiRotorControllerFeedback;
  using ros_message_type = tobas_debug_msgs::msg::MultiRotorControllerFeedback;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_position, dst.target_position);
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.target_orientation, dst.target_orientation);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_velocity_global, dst.target_velocity_global);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_velocity_local, dst.target_velocity_local);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_accel_global, dst.target_accel_global);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_accel_local, dst.target_accel_local);
    dst.target_thrust = src.target_thrust;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.position_integral_error, dst.position_integral_error);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_position, dst.target_position);
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.target_orientation, dst.target_orientation);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_velocity_global, dst.target_velocity_global);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_velocity_local, dst.target_velocity_local);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_accel_global, dst.target_accel_global);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_accel_local, dst.target_accel_local);
    dst.target_thrust = src.target_thrust;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.position_integral_error, dst.position_integral_error);
  }
};

namespace tobas_debug_msgs
{
using MultiRotorControllerFeedbackAdapter = rclcpp::
  TypeAdapter<tobas_debug_msgs::MultiRotorControllerFeedback, tobas_debug_msgs::msg::MultiRotorControllerFeedback>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_debug_msgs::MultiRotorControllerFeedback,
  tobas_debug_msgs::msg::MultiRotorControllerFeedback);
