#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl_msgs/msg/jnt_array_vel_stamped.hpp>

#include "./jnt_array_vel.hpp"

namespace tobas_kdl_msgs
{
struct JntArrayVelStamped
{
  std_msgs::msg::Header header;
  kdl::JntArrayVel q;

  using SharedPtr = std::shared_ptr<JntArrayVelStamped>;
  using ConstSharedPtr = std::shared_ptr<const JntArrayVelStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayVelStamped, tobas_kdl_msgs::msg::JntArrayVelStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::JntArrayVelStamped;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayVelStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayVelAdapter::convert_to_ros_message(src.q, dst.q);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayVelAdapter::convert_to_custom(src.q, dst.q);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayVelStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayVelStamped, tobas_kdl_msgs::msg::JntArrayVelStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas_kdl_msgs::JntArrayVelStamped,
  tobas_kdl_msgs::msg::JntArrayVelStamped);
