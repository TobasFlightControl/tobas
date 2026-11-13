// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/fixed_wing/vehicle_params.hpp>

#include <tobas_drone_msgs/msg/vehicle_parameters.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::VehicleParameters;
  using ros_message_type = tobas_drone_msgs::msg::VehicleParameters;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    _dst.wing_surface = _src.wing_surface;
    _dst.wing_span = _src.wing_span;
    _dst.mac = _src.mac;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(_src.ac, _dst.ac);
    _dst.alpha_limit.lower = _src.alpha_limit.lower;
    _dst.alpha_limit.upper = _src.alpha_limit.upper;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    _dst.wing_surface = _src.wing_surface;
    _dst.wing_span = _src.wing_span;
    _dst.mac = _src.mac;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(_src.ac, _dst.ac);
    _dst.alpha_limit.lower = _src.alpha_limit.lower;
    _dst.alpha_limit.upper = _src.alpha_limit.upper;
  }
};

namespace tobas_drone_msgs
{
using VehicleParametersAdapter =
  rclcpp::TypeAdapter<tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters);
