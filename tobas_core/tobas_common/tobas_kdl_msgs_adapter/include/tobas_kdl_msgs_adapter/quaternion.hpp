#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/quaternion.hpp>

#include <tobas_kdl_msgs/msg/quaternion.hpp>

template <>
struct rclcpp::TypeAdapter<kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion>
{
  using is_specialized = std::true_type;
  using custom_type = kdl::Quaternion;
  using ros_message_type = tobas_kdl_msgs::msg::Quaternion;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.w = src.w;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.x = src.x;
    dst.y = src.y;
    dst.z = src.z;
    dst.w = src.w;
  }
};

namespace tobas_kdl_msgs
{
using QuaternionAdapter = rclcpp::TypeAdapter<kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(kdl::Quaternion, tobas_kdl_msgs::msg::Quaternion);
