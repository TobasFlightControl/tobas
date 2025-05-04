#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl/wrench.hpp>

#include <tobas_kdl_msgs/msg/wrench_stamped.hpp>

#include "./wrench.hpp"

namespace tobas_kdl_msgs
{
struct WrenchStamped
{
  std_msgs::msg::Header header;
  kdl::Wrench wrench;

  using SharedPtr = std::shared_ptr<WrenchStamped>;
  using ConstSharedPtr = std::shared_ptr<const WrenchStamped>;
};
}  // namespace tobas_kdl_msgs

template <>
struct rclcpp::TypeAdapter<tobas_kdl_msgs::WrenchStamped, tobas_kdl_msgs::msg::WrenchStamped>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_kdl_msgs::WrenchStamped;
  using ros_message_type = tobas_kdl_msgs::msg::WrenchStamped;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::WrenchAdapter::convert_to_ros_message(src.wrench, dst.wrench);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    tobas_kdl_msgs::WrenchAdapter::convert_to_custom(src.wrench, dst.wrench);
  }
};

namespace tobas_kdl_msgs
{
using WrenchStampedAdapter = rclcpp::TypeAdapter<tobas_kdl_msgs::WrenchStamped, tobas_kdl_msgs::msg::WrenchStamped>;
}  // namespace tobas_kdl_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_kdl_msgs::WrenchStamped, tobas_kdl_msgs::msg::WrenchStamped);
