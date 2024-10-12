#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/Vector.hpp>

#include <tobas_msgs/msg/position_yaw.hpp>

namespace tobas_msgs
{
struct PositionYaw
{
  tobas_msgs::msg::CommandLevel level;
  kdl::Vector pos;
  double yaw;

  using SharedPtr = std::shared_ptr<PositionYaw>;
  using ConstSharedPtr = std::shared_ptr<const PositionYaw>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::PositionYaw, tobas_msgs::msg::PositionYaw>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::PositionYaw;
  using ros_message_type = tobas_msgs::msg::PositionYaw;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.pos, dst.pos);
    dst.yaw = src.yaw;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.level = src.level;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.pos, dst.pos);
    dst.yaw = src.yaw;
  }
};

namespace tobas_msgs
{
using PositionYawAdapter = rclcpp::TypeAdapter<tobas_msgs::PositionYaw, tobas_msgs::msg::PositionYaw>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::PositionYaw, tobas_msgs::msg::PositionYaw);
