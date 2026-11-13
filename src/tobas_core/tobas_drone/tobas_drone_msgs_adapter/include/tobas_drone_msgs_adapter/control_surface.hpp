// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

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

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.link_name = _src.link_name;

    _dst.c_lift_delta = _src.c_lift_delta;
    _dst.c_drag_abs_delta = _src.c_drag_abs_delta;
    _dst.c_side_delta = _src.c_side_delta;
    _dst.c_roll_delta = _src.c_roll_delta;
    _dst.c_pitch_delta = _src.c_pitch_delta;
    _dst.c_yaw_delta = _src.c_yaw_delta;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.link_name = _src.link_name;

    _dst.c_lift_delta = _src.c_lift_delta;
    _dst.c_drag_abs_delta = _src.c_drag_abs_delta;
    _dst.c_side_delta = _src.c_side_delta;
    _dst.c_roll_delta = _src.c_roll_delta;
    _dst.c_pitch_delta = _src.c_pitch_delta;
    _dst.c_yaw_delta = _src.c_yaw_delta;
  }
};

namespace tobas_drone_msgs
{
using ControlSurfaceAdapter = rclcpp::TypeAdapter<tobas::ControlSurface, tobas_drone_msgs::msg::ControlSurface>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::ControlSurface, tobas_drone_msgs::msg::ControlSurface);
