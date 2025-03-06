#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/pos_vel.hpp>

namespace tobas_command_msgs
{
struct PosVel
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Vector pos;
  kdl::Vector vel;

  using SharedPtr = std::shared_ptr<PosVel>;
  using ConstSharedPtr = std::shared_ptr<const PosVel>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::PosVel, tobas_command_msgs::msg::PosVel>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::PosVel;
  using ros_message_type = tobas_command_msgs::msg::PosVel;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.vel);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.vel, dst.vel);
  }
};

namespace tobas_command_msgs
{
using PosVelAdapter = rclcpp::TypeAdapter<tobas_command_msgs::PosVel, tobas_command_msgs::msg::PosVel>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::PosVel, tobas_command_msgs::msg::PosVel);
