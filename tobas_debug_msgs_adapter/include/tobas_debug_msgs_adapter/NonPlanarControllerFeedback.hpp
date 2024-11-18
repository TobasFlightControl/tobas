#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_kdl_msgs_adapter/euler.hpp>
#include <tobas_kdl_msgs_adapter/twist.hpp>
#include <tobas_kdl_msgs_adapter/accel.hpp>

#include <tobas_debug_msgs/msg/non_planar_controller_feedback.hpp>

namespace tobas_debug_msgs
{
struct NonPlanarControllerFeedback
{
  std_msgs::msg::Header header;
  kdl::Vector target_position;
  kdl::Euler target_orientation;
  kdl::Twist target_twist_global;
  kdl::Twist target_twist_local;
  kdl::Accel target_accel_global;
  kdl::Accel target_accel_local;
  kdl::Vector position_integral_error;
  kdl::Euler orientation_integral_error;

  using SharedPtr = std::shared_ptr<NonPlanarControllerFeedback>;
  using ConstSharedPtr = std::shared_ptr<const NonPlanarControllerFeedback>;
};
}  // namespace tobas_debug_msgs

template <>
struct rclcpp::
  TypeAdapter<tobas_debug_msgs::NonPlanarControllerFeedback, tobas_debug_msgs::msg::NonPlanarControllerFeedback>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_debug_msgs::NonPlanarControllerFeedback;
  using ros_message_type = tobas_debug_msgs::msg::NonPlanarControllerFeedback;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.target_position, dst.target_position);
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.target_orientation, dst.target_orientation);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.target_twist_global, dst.target_twist_global);
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.target_twist_local, dst.target_twist_local);
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.target_accel_global, dst.target_accel_global);
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.target_accel_local, dst.target_accel_local);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.position_integral_error, dst.position_integral_error);
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(
      src.orientation_integral_error, dst.orientation_integral_error);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.target_position, dst.target_position);
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.target_orientation, dst.target_orientation);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.target_twist_global, dst.target_twist_global);
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.target_twist_local, dst.target_twist_local);
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.target_accel_global, dst.target_accel_global);
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.target_accel_local, dst.target_accel_local);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.position_integral_error, dst.position_integral_error);
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.orientation_integral_error, dst.orientation_integral_error);
  }
};

namespace tobas_debug_msgs
{
using NonPlanarControllerFeedbackAdapter = rclcpp::
  TypeAdapter<tobas_debug_msgs::NonPlanarControllerFeedback, tobas_debug_msgs::msg::NonPlanarControllerFeedback>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_debug_msgs::NonPlanarControllerFeedback,
  tobas_debug_msgs::msg::NonPlanarControllerFeedback);
