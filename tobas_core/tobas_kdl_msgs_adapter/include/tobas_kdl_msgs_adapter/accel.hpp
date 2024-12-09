#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/accel.hpp>
#include <tobas_kdl_msgs/msg/accel.hpp>

#include "./vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Accel, tobas_kdl_msgs::msg::Accel>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Accel;
  using ros_message_type = tobas_kdl_msgs::msg::Accel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.linear, dst.linear);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.angular, dst.angular);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.linear, dst.linear);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.angular, dst.angular);
  }
};

namespace tobas_kdl_msgs
{
using AccelAdapter = rclcpp::TypeAdapter<kdl::Accel, tobas_kdl_msgs::msg::Accel>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Accel, tobas_kdl_msgs::msg::Accel);
