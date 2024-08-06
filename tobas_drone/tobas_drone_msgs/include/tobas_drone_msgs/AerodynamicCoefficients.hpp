#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/aerodynamic_coefs.hpp>
#include <tobas_drone_msgs/msg/aerodynamic_coefficients.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::AerodynamicCoefficients, tobas_drone_msgs::msg::AerodynamicCoefficients>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::AerodynamicCoefficients;
  using ros_message_type = tobas_drone_msgs::msg::AerodynamicCoefficients;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.c_lift_0 = dst.c_lift_0;
    dst.c_lift_alpha = dst.c_lift_alpha;

    dst.c_drag_0 = dst.c_drag_0;
    dst.c_drag_alpha = dst.c_drag_alpha;

    dst.c_side_beta = dst.c_side_beta;

    dst.c_roll_beta = dst.c_roll_beta;
    dst.c_roll_p = dst.c_roll_p;
    dst.c_roll_r = dst.c_roll_r;

    dst.c_pitch_0 = dst.c_pitch_0;
    dst.c_pitch_alpha = dst.c_pitch_alpha;
    dst.c_pitch_abs_beta = dst.c_pitch_abs_beta;
    dst.c_pitch_alpha_rate = dst.c_pitch_alpha_rate;
    dst.c_pitch_q = dst.c_pitch_q;

    dst.c_yaw_beta = dst.c_yaw_beta;
    dst.c_yaw_p = dst.c_yaw_p;
    dst.c_yaw_r = dst.c_yaw_r;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.c_lift_0 = dst.c_lift_0;
    dst.c_lift_alpha = dst.c_lift_alpha;

    dst.c_drag_0 = dst.c_drag_0;
    dst.c_drag_alpha = dst.c_drag_alpha;

    dst.c_side_beta = dst.c_side_beta;

    dst.c_roll_beta = dst.c_roll_beta;
    dst.c_roll_p = dst.c_roll_p;
    dst.c_roll_r = dst.c_roll_r;

    dst.c_pitch_0 = dst.c_pitch_0;
    dst.c_pitch_alpha = dst.c_pitch_alpha;
    dst.c_pitch_abs_beta = dst.c_pitch_abs_beta;
    dst.c_pitch_alpha_rate = dst.c_pitch_alpha_rate;
    dst.c_pitch_q = dst.c_pitch_q;

    dst.c_yaw_beta = dst.c_yaw_beta;
    dst.c_yaw_p = dst.c_yaw_p;
    dst.c_yaw_r = dst.c_yaw_r;
  }
};

namespace tobas_drone_msgs
{
using AerodynamicCoefficientsAdapter =
  rclcpp::TypeAdapter<tobas::AerodynamicCoefficients, tobas_drone_msgs::msg::AerodynamicCoefficients>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas::AerodynamicCoefficients,
  tobas_drone_msgs::msg::AerodynamicCoefficients);
