#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/wrench.hpp>
#include <tobas_kdl_msgs/msg/wrench.hpp>

#include "./Vector.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Wrench, tobas_kdl_msgs::msg::Wrench>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Wrench;
  using ros_message_type = tobas_kdl_msgs::msg::Wrench;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::Vector::convert_to_ros_message(src.force, dst.force);
    tobas_kdl_msgs::Vector::convert_to_ros_message(src.torque, dst.torque);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::Vector::convert_to_custom(src.force, dst.force);
    tobas_kdl_msgs::Vector::convert_to_custom(src.torque, dst.torque);
  }
};

namespace tobas_kdl_msgs
{
using Wrench = rclcpp::TypeAdapter<kdl::Wrench, tobas_kdl_msgs::msg::Wrench>;
}
