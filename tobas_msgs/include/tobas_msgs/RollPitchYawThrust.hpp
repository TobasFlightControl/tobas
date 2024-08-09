#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs/Euler.hpp>

#include <tobas_msgs/msg/roll_pitch_yaw_thrust.hpp>

namespace tobas_msgs
{
struct RollPitchYawThrust
{
  tobas_msgs::msg::CommandLevel level;
  kdl::Euler rpy;
  double thrust;

  using SharedPtr = std::shared_ptr<RollPitchYawThrust>;
  using ConstSharedPtr = std::shared_ptr<const RollPitchYawThrust>;
};
}  // namespace tobas_msgs

template <>
struct rclcpp::TypeAdapter<tobas_msgs::RollPitchYawThrust, tobas_msgs::msg::RollPitchYawThrust>
{
  using is_specialized = std::true_type;
  using custom_type = tobas_msgs::RollPitchYawThrust;
  using ros_message_type = tobas_msgs::msg::RollPitchYawThrust;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_ros_message(src.rpy, dst.rpy);
    dst.thrust = src.thrust;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.level = src.level;
    tobas_kdl_msgs::EulerAdapter::convert_to_custom(src.rpy, dst.rpy);
    dst.thrust = src.thrust;
  }
};

namespace tobas_msgs
{
using RollPitchYawThrustAdapter =
  rclcpp::TypeAdapter<tobas_msgs::RollPitchYawThrust, tobas_msgs::msg::RollPitchYawThrust>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas_msgs::RollPitchYawThrust, tobas_msgs::msg::RollPitchYawThrust);
