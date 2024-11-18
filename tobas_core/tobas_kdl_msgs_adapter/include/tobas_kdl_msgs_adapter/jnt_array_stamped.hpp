#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl_msgs/msg/jnt_array_stamped.hpp>

#include "./jnt_array.hpp"

namespace tobas_kdl_msgs
{
struct JntArrayStamped
{
  std_msgs::msg::Header header;
  kdl::JntArray q;

  using SharedPtr = std::shared_ptr<JntArrayStamped>;
  using ConstSharedPtr = std::shared_ptr<const JntArrayStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayStamped, tobas_kdl_msgs::msg::JntArrayStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::JntArrayStamped;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayAdapter::convert_to_ros_message(src.q, dst.q);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayAdapter::convert_to_custom(src.q, dst.q);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayStamped, tobas_kdl_msgs::msg::JntArrayStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::JntArrayStamped, tobas_kdl_msgs::msg::JntArrayStamped);
