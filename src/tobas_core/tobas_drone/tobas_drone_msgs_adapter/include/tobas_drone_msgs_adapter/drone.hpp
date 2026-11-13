// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>

#include <rclcpp/logging.hpp>
#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/drone.hpp>

#include <tobas_drone_msgs/msg/drone.hpp>

#include "./electric_propulsion_system_config.hpp"
#include "./fixed_wing_config.hpp"
#include "./ice_propulsion_system_config.hpp"
#include "./joint_config.hpp"
#include "./pwm_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::Drone;
  using ros_message_type = tobas_drone_msgs::msg::Drone;

  static void convert_to_ros_message(const custom_type& _src, ros_message_type& _dst)
  {
    // Name
    _dst.name = _src.name;

    // Joint
    _dst.joints.clear();
    for (const auto& [_, joint] : _src.joints) {
      _dst.joints.emplace_back();
      tobas_drone_msgs::JointConfigAdapter::convert_to_ros_message(joint, _dst.joints.back());
    }

    // PWM
    _dst.pwms.clear();
    for (const auto& [_, pwm] : _src.pwms) {
      _dst.pwms.emplace_back();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_ros_message(pwm, _dst.pwms.back());
    }

    // Propulsion System
    if (_src.prop) {
      const auto prop_type = _src.prop->type();
      _dst.prop_type = static_cast<int8_t>(prop_type);
      switch (prop_type) {
        case tobas::PropulsionSystem::kElectric: {
          const auto eprop = std::static_pointer_cast<tobas::ElectricPropulsionSystemConfig>(_src.prop);
          tobas_drone_msgs::ElectricPropulsionSystemConfigAdapter::convert_to_ros_message(*eprop, _dst.eprop);
          break;
        }
        case tobas::PropulsionSystem::kIce: {
          const auto iprop = std::static_pointer_cast<tobas::IcePropulsionSystemConfig>(_src.prop);
          tobas_drone_msgs::IcePropulsionSystemConfigAdapter::convert_to_ros_message(*iprop, _dst.iprop);
          break;
        }
        default: {
          RCLCPP_ERROR_STREAM(getLogger(), "Invalid propulsion system type: " << (int)prop_type);
          _dst.prop_type = -1;
          break;
        }
      }
    }
    else {
      _dst.prop_type = -1;
    }

    // Fixed Wing
    if (_src.fixed_wing) {
      _dst.has_fixed_wing = true;
      tobas_drone_msgs::FixedWingConfigAdapter::convert_to_ros_message(*_src.fixed_wing, _dst.fixed_wing);
    }
    else {
      _dst.has_fixed_wing = false;
    }

    // S.BUS Channels
    _dst.num_sbus_channels = _src.num_sbus_channels;
  }

  static void convert_to_custom(const ros_message_type& _src, custom_type& _dst)
  {
    // Name
    _dst.name = _src.name;

    // Joint
    _dst.joints.clear();
    for (const auto& joint : _src.joints) {
      _dst.joints[joint.name] = tobas::JointConfig();
      tobas_drone_msgs::JointConfigAdapter::convert_to_custom(joint, _dst.joints.at(joint.name));
    }

    // PWM
    _dst.pwms.clear();
    for (const auto& pwm : _src.pwms) {
      _dst.pwms[pwm.name] = tobas::PwmConfig();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_custom(pwm, _dst.pwms.at(pwm.name));
    }

    // Propulsion System
    if (_src.prop_type >= 0) {
      switch (static_cast<tobas::PropulsionSystem>(_src.prop_type)) {
        case tobas::PropulsionSystem::kElectric: {
          const auto eprop = std::make_shared<tobas::ElectricPropulsionSystemConfig>();
          tobas_drone_msgs::ElectricPropulsionSystemConfigAdapter::convert_to_custom(_src.eprop, *eprop);
          _dst.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(eprop);
          break;
        }
        case tobas::PropulsionSystem::kIce: {
          const auto iprop = std::make_shared<tobas::IcePropulsionSystemConfig>();
          tobas_drone_msgs::IcePropulsionSystemConfigAdapter::convert_to_custom(_src.iprop, *iprop);
          _dst.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(iprop);
          break;
        }
        default: {
          RCLCPP_ERROR_STREAM(getLogger(), "Invalid propulsion system type: " << (int)_src.prop_type);
          _dst.prop.reset();
          break;
        }
      }
    }
    else {
      _dst.prop.reset();
    }

    // Fixed Wing
    if (_src.has_fixed_wing) {
      _dst.fixed_wing = std::make_shared<tobas::FixedWingConfig>();
      tobas_drone_msgs::FixedWingConfigAdapter::convert_to_custom(_src.fixed_wing, *_dst.fixed_wing);
    }
    else {
      _dst.fixed_wing.reset();
    }

    // S.BUS Channels
    _dst.num_sbus_channels = _src.num_sbus_channels;
  }

private:
  static rclcpp::Logger getLogger()
  {
    return get_logger("drone_adapter");
  }
};

namespace tobas_drone_msgs
{
using DroneAdapter = rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>;
}  // namespace tobas_drone_msgs

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::Drone, tobas_drone_msgs::msg::Drone);
