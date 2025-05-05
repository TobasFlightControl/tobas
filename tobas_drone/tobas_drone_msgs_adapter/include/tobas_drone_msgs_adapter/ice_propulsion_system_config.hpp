#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/propulsion_system/ice_propulsion_system/ice_propulsion_system.hpp>

#include <tobas_drone_msgs/msg/ice_propulsion_system_config.hpp>

#include "./engine_config.hpp"
#include "./ice_rotor_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::ICEPropulsionSystemConfig, tobas_drone_msgs::msg::ICEPropulsionSystemConfig>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::ICEPropulsionSystemConfig;
  using ros_message_type = tobas_drone_msgs::msg::ICEPropulsionSystemConfig;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    tobas_drone_msgs::EngineConfigAdapter::convert_to_ros_message(src.engine, dst.engine);

    dst.rotors.clear();
    for (const auto& [_, src_rotor] : src.rotors) {
      const auto src_irotor = boost::polymorphic_pointer_downcast<tobas::ICERotorConfig>(src_rotor);
      dst.rotors.emplace_back();
      tobas_drone_msgs::ICERotorConfigAdapter::convert_to_ros_message(*src_irotor, dst.rotors.back());
    }
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    tobas_drone_msgs::EngineConfigAdapter::convert_to_custom(src.engine, dst.engine);

    dst.rotors.clear();
    for (const auto& src_irotor : src.rotors) {
      const auto dst_irotor = std::make_shared<tobas::ICERotorConfig>();
      tobas_drone_msgs::ICERotorConfigAdapter::convert_to_custom(src_irotor, *dst_irotor);
      dst.rotors[src_irotor.rotor.link_name] = dst_irotor;
    }
  }
};

namespace tobas_drone_msgs
{
using ICEPropulsionSystemConfigAdapter =
  rclcpp::TypeAdapter<tobas::ICEPropulsionSystemConfig, tobas_drone_msgs::msg::ICEPropulsionSystemConfig>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(
  tobas::ICEPropulsionSystemConfig,
  tobas_drone_msgs::msg::ICEPropulsionSystemConfig);
