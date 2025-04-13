#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>

#include "./joint_config.hpp"
#include "./pwm_config.hpp"
#include "./electric_propulsion_system_config.hpp"
#include "./ice_propulsion_system_config.hpp"
#include "./fixed_wing_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::Drone;
  using ros_message_type = tobas_drone_msgs::msg::Drone;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    // Name
    dst.name = src.name;

    // Joint
    dst.joints.clear();
    for (const auto& [_, joint] : src.joints)
    {
      dst.joints.emplace_back();
      tobas_drone_msgs::JointConfigAdapter::convert_to_ros_message(joint, dst.joints.back());
    }

    // PWM
    dst.pwms.clear();
    for (const auto& [_, pwm] : src.pwms)
    {
      dst.pwms.emplace_back();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_ros_message(pwm, dst.pwms.back());
    }

    // Propulsion System
    if (src.prop)
    {
      dst.prop_type = static_cast<int8_t>(src.prop->type());
      switch (src.prop->type())
      {
        case tobas::propulsion_system_t::ELECTRIC:
        {
          const auto eprop = boost::polymorphic_pointer_downcast<tobas::ElectricPropulsionSystemConfig>(src.prop);
          tobas_drone_msgs::ElectricPropulsionSystemConfigAdapter::convert_to_ros_message(*eprop, dst.eprop);
          break;
        }
        case tobas::propulsion_system_t::ICE:
        {
          const auto iprop = boost::polymorphic_pointer_downcast<tobas::ICEPropulsionSystemConfig>(src.prop);
          tobas_drone_msgs::ICEPropulsionSystemConfigAdapter::convert_to_ros_message(*iprop, dst.iprop);
          break;
        }
        default:
        {
          std::cerr << "Invalid propulsion system type: " << (int)src.prop->type() << std::endl;
          dst.prop_type = -1;
          break;
        }
      }
    }
    else
    {
      dst.prop_type = -1;
    }

    // Fixed Wing
    if (src.fixed_wing)
    {
      dst.has_fixed_wing = true;
      tobas_drone_msgs::FixedWingConfigAdapter::convert_to_ros_message(*src.fixed_wing, dst.fixed_wing);
    }
    else
    {
      dst.has_fixed_wing = false;
    }

    // S.BUS Channels
    dst.num_sbus_channels = src.num_sbus_channels;
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    // Name
    dst.name = src.name;

    // Joint
    dst.joints.clear();
    for (const auto& joint : src.joints)
    {
      dst.joints[joint.name] = tobas::JointConfig();
      tobas_drone_msgs::JointConfigAdapter::convert_to_custom(joint, dst.joints.at(joint.name));
    }

    // PWM
    dst.pwms.clear();
    for (const auto& pwm : src.pwms)
    {
      dst.pwms[pwm.name] = tobas::PwmConfig();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_custom(pwm, dst.pwms.at(pwm.name));
    }

    // Propulsion System
    if (src.prop_type >= 0)
    {
      switch (static_cast<tobas::propulsion_system_t>(src.prop_type))
      {
        case tobas::propulsion_system_t::ELECTRIC:
        {
          const auto eprop = std::make_shared<tobas::ElectricPropulsionSystemConfig>();
          tobas_drone_msgs::ElectricPropulsionSystemConfigAdapter::convert_to_custom(src.eprop, *eprop);
          dst.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(eprop);
          break;
        }
        case tobas::propulsion_system_t::ICE:
        {
          const auto iprop = std::make_shared<tobas::ICEPropulsionSystemConfig>();
          tobas_drone_msgs::ICEPropulsionSystemConfigAdapter::convert_to_custom(src.iprop, *iprop);
          dst.prop = std::static_pointer_cast<tobas::PropulsionSystemConfig>(iprop);
          break;
        }
        default:
        {
          std::cerr << "Invalid propulsion system type: " << (int)src.prop_type << std::endl;
          dst.prop.reset();
          break;
        }
      }
    }
    else
    {
      dst.prop.reset();
    }

    // Fixed Wing
    if (src.has_fixed_wing)
    {
      dst.fixed_wing = std::make_shared<tobas::FixedWingConfig>();
      tobas_drone_msgs::FixedWingConfigAdapter::convert_to_custom(src.fixed_wing, *dst.fixed_wing);
    }
    else
    {
      dst.fixed_wing.reset();
    }

    // S.BUS Channels
    dst.num_sbus_channels = src.num_sbus_channels;
  }
};

namespace tobas_drone_msgs
{
using DroneAdapter = rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::Drone, tobas_drone_msgs::msg::Drone);
