#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_command_msgs/msg/pos_vel_acc_yaw.hpp>

namespace tobas_command_msgs
{
struct PosVelAccYaw
{
  std_msgs::msg::Header header;
  tobas_command_msgs::msg::CommandLevel level;
  kdl::Vector pos;
  kdl::Vector vel;
  kdl::Vector acc;
  double yaw;

  using SharedPtr = std::shared_ptr<PosVelAccYaw>;
  using ConstSharedPtr = std::shared_ptr<const PosVelAccYaw>;
};
}  // namespace tobas_command_msgs

template <>
struct rclcpp::TypeAdapter<tobas_command_msgs::PosVelAccYaw, tobas_command_msgs::msg::PosVelAccYaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_command_msgs::PosVelAccYaw;
  using ros_message_type = tobas_command_msgs::msg::PosVelAccYaw;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.acc, dst.acc);
    dst.yaw = src.yaw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.acc, dst.acc);
    dst.yaw = src.yaw;
  }
};

namespace tobas_command_msgs
{
using PosVelAccYawAdapter =
  rclcpp::TypeAdapter<tobas_command_msgs::PosVelAccYaw, tobas_command_msgs::msg::PosVelAccYaw>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_command_msgs::PosVelAccYaw, tobas_command_msgs::msg::PosVelAccYaw);
