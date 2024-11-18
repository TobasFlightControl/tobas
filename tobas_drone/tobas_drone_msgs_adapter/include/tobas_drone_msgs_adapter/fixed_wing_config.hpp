#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>

#include <tobas_drone_core/fixed_wing.hpp>
#include <tobas_drone_msgs/msg/fixed_wing_config.hpp>

#include "./vehicle_parameters.hpp"
#include "./aerodynamic_coefficients.hpp"
#include "./control_surface.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::FixedWingConfig;
  using ros_message_type = tobas_drone_msgs::msg::FixedWingConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::VehicleParametersAdapter::convert_to_ros_message(src.vehicle, dst.vehicle);
    tobas_drone_msgs::AerodynamicCoefficientsAdapter::convert_to_ros_message(src.aerodynamics, dst.aerodynamics);

    dst.control_surfaces.clear();
    for (const auto& [_, cs] : src.control_surfaces)
    {
      dst.control_surfaces.emplace_back();
      tobas_drone_msgs::ControlSurfaceAdapter::convert_to_ros_message(cs, dst.control_surfaces.back());
    }
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::VehicleParametersAdapter::convert_to_custom(src.vehicle, dst.vehicle);
    tobas_drone_msgs::AerodynamicCoefficientsAdapter::convert_to_custom(src.aerodynamics, dst.aerodynamics);

    dst.control_surfaces.clear();
    for (const auto& cs : src.control_surfaces)
    {
      dst.control_surfaces[cs.channel] = tobas::ControlSurface();
      tobas_drone_msgs::ControlSurfaceAdapter::convert_to_custom(cs, dst.control_surfaces.at(cs.channel));
    }
  }
};

namespace tobas_drone_msgs
{
using FixedWingConfigAdapter = rclcpp::TypeAdapter<tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::FixedWingConfig, tobas_drone_msgs::msg::FixedWingConfig);
