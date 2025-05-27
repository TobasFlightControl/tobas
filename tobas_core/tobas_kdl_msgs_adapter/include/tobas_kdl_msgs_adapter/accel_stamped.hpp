#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/accel.hpp>

#include <tobas_kdl_msgs/msg/accel_stamped.hpp>

#include "./accel.hpp"

namespace tobas_kdl_msgs
{
struct AccelStamped
{
  std_msgs::msg::Header header;
  kdl::Accel accel;

  using SharedPtr = std::shared_ptr<AccelStamped>;
  using ConstSharedPtr = std::shared_ptr<const AccelStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::AccelStamped, tobas_kdl_msgs::msg::AccelStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::AccelStamped;
  using ros_message_type = tobas_kdl_msgs::msg::AccelStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::AccelAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::AccelAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_kdl_msgs
{
using AccelStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::AccelStamped, tobas_kdl_msgs::msg::AccelStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::AccelStamped, tobas_kdl_msgs::msg::AccelStamped);
