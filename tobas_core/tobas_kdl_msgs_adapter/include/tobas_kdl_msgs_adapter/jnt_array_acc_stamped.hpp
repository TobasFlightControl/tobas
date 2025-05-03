#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/jntarray.hpp>
#include <tobas_kdl_msgs/msg/jnt_array_acc_stamped.hpp>

#include "./jnt_array_acc.hpp"

namespace tobas_kdl_msgs
{
struct JntArrayAccStamped
{
  std_msgs::msg::Header header;
  kdl::JntArrayAcc q;

  using SharedPtr = std::shared_ptr<JntArrayAccStamped>;
  using ConstSharedPtr = std::shared_ptr<const JntArrayAccStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayAccStamped, tobas_kdl_msgs::msg::JntArrayAccStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::JntArrayAccStamped;
  using ros_message_type = tobas_kdl_msgs::msg::JntArrayAccStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayAccAdapter::convert_to_ros_message(src.q, dst.q);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::JntArrayAccAdapter::convert_to_custom(src.q, dst.q);
  }
};

namespace tobas_kdl_msgs
{
using JntArrayAccStampedAdapter =
  rclcpp::TypeAdapter<tobas_kdl_msgs::JntArrayAccStamped, tobas_kdl_msgs::msg::JntArrayAccStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::JntArrayAccStamped, tobas_kdl_msgs::msg::JntArrayAccStamped);
