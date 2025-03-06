#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/accel.hpp>

namespace tobas_command_msgs
{
struct Accel
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Vector accel;

  using SharedPtr = std::shared_ptr<Accel>;
  using ConstSharedPtr = std::shared_ptr<const Accel>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::Accel, tobas_command_msgs::msg::Accel>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::Accel;
  using ros_message_type = tobas_command_msgs::msg::Accel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.accel, dst.accel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.accel, dst.accel);
  }
};

namespace tobas_command_msgs
{
using AccelAdapter = rclcpp::TypeAdapter<tobas_command_msgs::Accel, tobas_command_msgs::msg::Accel>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::Accel, tobas_command_msgs::msg::Accel);
