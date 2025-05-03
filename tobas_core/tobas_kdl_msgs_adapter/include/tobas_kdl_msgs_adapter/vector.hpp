#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/vector.hpp>
#include <tobas_kdl_msgs/msg/vector.hpp>

template <>
struct rclcpp::TypeAdapter<kdl::Vector, tobas_kdl_msgs::msg::Vector>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Vector;
  using ros_message_type = tobas_kdl_msgs::msg::Vector;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.x = src.x();
    dst.y = src.y();
    dst.z = src.z();
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.x(src.x);
    dst.y(src.y);
    dst.z(src.z);
  }
};

namespace tobas_kdl_msgs
{
using VectorAdapter = rclcpp::TypeAdapter<kdl::Vector, tobas_kdl_msgs::msg::Vector>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Vector, tobas_kdl_msgs::msg::Vector);
