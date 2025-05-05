#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/fixed_wing/vehicle_params.hpp>

#include <tobas_kdl_msgs_adapter/vector.hpp>
#include <tobas_drone_msgs/msg/vehicle_parameters.hpp>

template <>
struct rclcpp::TypeAdapter<tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::VehicleParameters;
  using ros_message_type = tobas_drone_msgs::msg::VehicleParameters;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.wing_surface = src.wing_surface;
    dst.wing_span = src.wing_span;
    dst.mac = src.mac;
    tobas_kdl_msgs::VectorAdapter::convert_to_ros_message(src.ac, dst.ac);
    dst.alpha_limit.lower = src.alpha_limit.lower;
    dst.alpha_limit.upper = src.alpha_limit.upper;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.wing_surface = src.wing_surface;
    dst.wing_span = src.wing_span;
    dst.mac = src.mac;
    tobas_kdl_msgs::VectorAdapter::convert_to_custom(src.ac, dst.ac);
    dst.alpha_limit.lower = src.alpha_limit.lower;
    dst.alpha_limit.upper = src.alpha_limit.upper;
  }
};

namespace tobas_drone_msgs
{
using VehicleParametersAdapter =
  rclcpp::TypeAdapter<tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::VehicleParameters, tobas_drone_msgs::msg::VehicleParameters);
