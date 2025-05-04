#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/quaternion.hpp>

#include <tobas_kdl_msgs/msg/quaternion_stamped.hpp>

#include "./quaternion.hpp"

namespace tobas_kdl_msgs
{
struct QuaternionStamped
{
  std_msgs::msg::Header header;
  kdl::Quaternion quaternion;

  using SharedPtr = std::shared_ptr<QuaternionStamped>;
  using ConstSharedPtr = std::shared_ptr<const QuaternionStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::QuaternionStamped, tobas_kdl_msgs::msg::QuaternionStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::QuaternionStamped;
  using ros_message_type = tobas_kdl_msgs::msg::QuaternionStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::QuaternionAdapter::convert_to_ros_message(src.quaternion, dst.quaternion);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::QuaternionAdapter::convert_to_custom(src.quaternion, dst.quaternion);
  }
};

namespace tobas_kdl_msgs
{
using QuaternionStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::QuaternionStamped, tobas_kdl_msgs::msg::QuaternionStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::QuaternionStamped, tobas_kdl_msgs::msg::QuaternionStamped);
