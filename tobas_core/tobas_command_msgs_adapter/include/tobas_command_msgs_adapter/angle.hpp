#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_command_msgs/msg/angle.hpp>

namespace tobas_command_msgs
{
struct Angle
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Euler angle;

  using SharedPtr = std::shared_ptr<Angle>;
  using ConstSharedPtr = std::shared_ptr<const Angle>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::Angle;
  using ros_message_type = tobas_command_msgs::msg::Angle;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.angle, dst.angle);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.angle, dst.angle);
  }
};

namespace tobas_command_msgs
{
using AngleAdapter = rclcpp::TypeAdapter<tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle>;
}  // namespace tobas_command_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::Angle, tobas_command_msgs::msg::Angle);
