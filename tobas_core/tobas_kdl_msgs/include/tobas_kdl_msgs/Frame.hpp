#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/frame.hpp>
#include <tobas_kdl_msgs/msg/frame.hpp>

#include "./Vector.hpp"
#include "./Rotation.hpp"

template <>
struct rclcpp::TypeAdapter<kdl::Frame, tobas_kdl_msgs::msg::Frame>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Frame;
  using ros_message_type = tobas_kdl_msgs::msg::Frame;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_kdl_msgs::Vector::convert_to_ros_message(src.p, dst.trans);
    tobas_kdl_msgs::Rotation::convert_to_ros_message(src.M, dst.rot);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_kdl_msgs::Vector::convert_to_custom(src.trans, dst.p);
    tobas_kdl_msgs::Rotation::convert_to_custom(src.rot, dst.M);
  }
};

namespace tobas_kdl_msgs
{
using Frame = rclcpp::TypeAdapter<kdl::Frame, tobas_kdl_msgs::msg::Frame>;
}
