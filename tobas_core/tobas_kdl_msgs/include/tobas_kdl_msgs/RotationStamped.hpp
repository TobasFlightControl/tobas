#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/rotation.hpp>
#include <tobas_kdl_msgs/msg/rotation_stamped.hpp>

#include "./Rotation.hpp"

namespace tobas_kdl_msgs
{
struct RotationStamped
{
  std_msgs::msg::Header header;
  kdl::Rotation rotation;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::RotationStamped, tobas_kdl_msgs::msg::RotationStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::RotationStamped;
  using ros_message_type = tobas_kdl_msgs::msg::RotationStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RotationAdapter::convert_to_ros_message(src.rotation, dst.rotation);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::RotationAdapter::convert_to_custom(src.rotation, dst.rotation);
  }
};

namespace tobas_kdl_msgs
{
using RotationStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::RotationStamped, tobas_kdl_msgs::msg::RotationStamped>;
}
