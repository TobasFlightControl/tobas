// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/fixed_wing/fixed_wing.hpp>

#include <tobas_drone_msgs/msg/fixed_wing_config.hpp>
#include <tobas_kdl_msgs_adapter/vector.hpp>

#include "./aerodynamic_coefficients.hpp"
#include "./control_surface.hpp"
#include "./vehicle_parameters.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::FixedWingConfig;
  using ros_message_type = tobas_drone_msgs::msg::FixedWingConfig;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    tobas_drone_msgs::VehicleParametersAdapter::convert_to_ros_message(_src.vehicle, _dst.vehicle);
    tobas_drone_msgs::AerodynamicCoefficientsAdapter::convert_to_ros_message(_src.aerodynamics, _dst.aerodynamics);

    _dst.control_surfaces.clear();
    for (const auto& [_, cs] : _src.control_surfaces) {
      _dst.control_surfaces.emplace_back();
      tobas_drone_msgs::ControlSurfaceAdapter::convert_to_ros_message(cs, _dst.control_surfaces.back());
    }
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    tobas_drone_msgs::VehicleParametersAdapter::convert_to_custom(_src.vehicle, _dst.vehicle);
    tobas_drone_msgs::AerodynamicCoefficientsAdapter::convert_to_custom(_src.aerodynamics, _dst.aerodynamics);

    _dst.control_surfaces.clear();
    for (const auto& cs : _src.control_surfaces) {
      _dst.control_surfaces[cs.link_name] = tobas::ControlSurface();
      tobas_drone_msgs::ControlSurfaceAdapter::convert_to_custom(cs, _dst.control_surfaces.at(cs.link_name));
    }
  }
};

namespace tobas_drone_msgs
{
using FixedWingConfigAdapter = rclcpp::TypeAdapter<tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig);
