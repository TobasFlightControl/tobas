#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/fixed_wing/control_surface.hpp>

#include <tobas_drone_msgs/msg/control_surface.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::ControlSurface, tobas_drone_msgs::msg::ControlSurface>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::ControlSurface;
  using ros_message_type = tobas_drone_msgs::msg::ControlSurface;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.channel = src.channel;
    dst.link_name = src.link_name;

    dst.c_lift_delta = src.c_lift_delta;
    dst.c_drag_abs_delta = src.c_drag_abs_delta;
    dst.c_side_delta = src.c_side_delta;
    dst.c_roll_delta = src.c_roll_delta;
    dst.c_pitch_delta = src.c_pitch_delta;
    dst.c_yaw_delta = src.c_yaw_delta;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.channel = src.channel;
    dst.link_name = src.link_name;

    dst.c_lift_delta = src.c_lift_delta;
    dst.c_drag_abs_delta = src.c_drag_abs_delta;
    dst.c_side_delta = src.c_side_delta;
    dst.c_roll_delta = src.c_roll_delta;
    dst.c_pitch_delta = src.c_pitch_delta;
    dst.c_yaw_delta = src.c_yaw_delta;
  }
};

namespace tobas_drone_msgs
{
using ControlSurfaceAdapter = rclcpp::TypeAdapter<tobas::ControlSurface, tobas_drone_msgs::msg::ControlSurface>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::ControlSurface, tobas_drone_msgs::msg::ControlSurface);
