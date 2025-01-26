#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_kdl_msgs_adapter/euler.hpp>

#include <tobas_msgs/msg/pose_twist_accel_command.hpp>

namespace tobas_msgs
{
struct PoseTwistAccelCommand
{
  std_msgs::msg::Header header;
  tobas_msgs::msg::CommandLevel level;
  tobas_msgs::msg::FrameId frame_id;
  kdl::Vector pos;
  kdl::Vector vel;
  kdl::Vector acc;
  kdl::Euler rpy;
  kdl::Vector gyro;
  kdl::Vector dgyro;

  using SharedPtr = std::shared_ptr<PoseTwistAccelCommand>;
  using ConstSharedPtr = std::shared_ptr<const PoseTwistAccelCommand>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::PoseTwistAccelCommand, tobas_msgs::msg::PoseTwistAccelCommand>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::PoseTwistAccelCommand;
  using ros_message_type = tobas_msgs::msg::PoseTwistAccelCommand;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    dst.frame_id = src.frame_id;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.acc, dst.acc);
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.rpy, dst.rpy);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.dgyro, dst.dgyro);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.header = src.header;
    dst.level = src.level;
    dst.frame_id = src.frame_id;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.pos, dst.pos);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.vel, dst.vel);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.acc, dst.acc);
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.rpy, dst.rpy);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.gyro, dst.gyro);
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.dgyro, dst.dgyro);
  }
};

namespace tobas_msgs
{
using PoseTwistAccelCommandAdapter =
  rclcpp::TypeAdapter<tobas_msgs::PoseTwistAccelCommand, tobas_msgs::msg::PoseTwistAccelCommand>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::PoseTwistAccelCommand, tobas_msgs::msg::PoseTwistAccelCommand);
