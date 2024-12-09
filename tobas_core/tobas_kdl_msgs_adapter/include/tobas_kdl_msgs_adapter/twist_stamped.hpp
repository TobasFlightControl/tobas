#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/twist.hpp>
#include <tobas_kdl_msgs/msg/twist_stamped.hpp>

#include "./twist.hpp"

namespace tobas_kdl_msgs
{
struct TwistStamped
{
  std_msgs::msg::Header header;
  kdl::Twist twist;

  using SharedPtr = std::shared_ptr<TwistStamped>;
  using ConstSharedPtr = std::shared_ptr<const TwistStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::TwistStamped, tobas_kdl_msgs::msg::TwistStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::TwistStamped;
  using ros_message_type = tobas_kdl_msgs::msg::TwistStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::TwistAdapter::convert_to_ros_message(src.twist, dst.twist);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::TwistAdapter::convert_to_custom(src.twist, dst.twist);
  }
};

namespace tobas_kdl_msgs
{
using TwistStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::TwistStamped, tobas_kdl_msgs::msg::TwistStamped>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::TwistStamped, tobas_kdl_msgs::msg::TwistStamped);
