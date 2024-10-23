#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/euler.hpp>
#include <tobas_kdl_msgs/msg/euler_stamped.hpp>

#include "./Euler.hpp"

namespace tobas_kdl_msgs
{
struct EulerStamped
{
  std_msgs::msg::Header header;
  kdl::Euler euler;

  using SharedPtr = std::shared_ptr<EulerStamped>;
  using ConstSharedPtr = std::shared_ptr<const EulerStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::EulerStamped, tobas_kdl_msgs::msg::EulerStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::EulerStamped;
  using ros_message_type = tobas_kdl_msgs::msg::EulerStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.euler, dst.euler);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.euler, dst.euler);
  }
};

namespace tobas_kdl_msgs
{
using EulerStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::EulerStamped, tobas_kdl_msgs::msg::EulerStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::EulerStamped, tobas_kdl_msgs::msg::EulerStamped);
