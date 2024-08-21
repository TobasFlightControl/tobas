#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/twist.hpp>
#include <tobas_kdl_msgs/msg/twist.hpp>

#include "./Vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Twist, tobas_kdl_msgs::msg::Twist>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Twist;
  using ros_message_type = tobas_kdl_msgs::msg::Twist;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.linear);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.rot, dst.angular);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.linear, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.angular, dst.rot);
  }
};

namespace tobas_kdl_msgs
{
using TwistAdapter = rclcpp::TypeAdapter<kdl::Twist, tobas_kdl_msgs::msg::Twist>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Twist, tobas_kdl_msgs::msg::Twist);
