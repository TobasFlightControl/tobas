// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/fixed_wing/aerodynamic_coefs.hpp>

#include <tobas_drone_msgs/msg/aerodynamic_coefficients.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::AerodynamicCoefficients, tobas_drone_msgs::msg::AerodynamicCoefficients>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::AerodynamicCoefficients;
  using ros_message_type = tobas_drone_msgs::msg::AerodynamicCoefficients;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.c_lift_0 = _src.c_lift_0;
    _dst.c_lift_alpha = _src.c_lift_alpha;

    _dst.c_drag_0 = _src.c_drag_0;
    _dst.c_drag_alpha = _src.c_drag_alpha;

    _dst.c_side_beta = _src.c_side_beta;

    _dst.c_roll_beta = _src.c_roll_beta;
    _dst.c_roll_p = _src.c_roll_p;
    _dst.c_roll_r = _src.c_roll_r;

    _dst.c_pitch_0 = _src.c_pitch_0;
    _dst.c_pitch_alpha = _src.c_pitch_alpha;
    _dst.c_pitch_abs_beta = _src.c_pitch_abs_beta;
    _dst.c_pitch_alpha_rate = _src.c_pitch_alpha_rate;
    _dst.c_pitch_q = _src.c_pitch_q;

    _dst.c_yaw_beta = _src.c_yaw_beta;
    _dst.c_yaw_p = _src.c_yaw_p;
    _dst.c_yaw_r = _src.c_yaw_r;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.c_lift_0 = _src.c_lift_0;
    _dst.c_lift_alpha = _src.c_lift_alpha;

    _dst.c_drag_0 = _src.c_drag_0;
    _dst.c_drag_alpha = _src.c_drag_alpha;

    _dst.c_side_beta = _src.c_side_beta;

    _dst.c_roll_beta = _src.c_roll_beta;
    _dst.c_roll_p = _src.c_roll_p;
    _dst.c_roll_r = _src.c_roll_r;

    _dst.c_pitch_0 = _src.c_pitch_0;
    _dst.c_pitch_alpha = _src.c_pitch_alpha;
    _dst.c_pitch_abs_beta = _src.c_pitch_abs_beta;
    _dst.c_pitch_alpha_rate = _src.c_pitch_alpha_rate;
    _dst.c_pitch_q = _src.c_pitch_q;

    _dst.c_yaw_beta = _src.c_yaw_beta;
    _dst.c_yaw_p = _src.c_yaw_p;
    _dst.c_yaw_r = _src.c_yaw_r;
  }
};

namespace tobas_drone_msgs
{
using AerodynamicCoefficientsAdapter =
  rclcpp::TypeAdapter<tobas::AerodynamicCoefficients, tobas_drone_msgs::msg::AerodynamicCoefficients>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas::AerodynamicCoefficients,
  tobas_drone_msgs::msg::AerodynamicCoefficients);
