#pragma once

#include <rclcpp/type_adapter.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_msgs/msg/drone.hpp>

#include "./battery_config.hpp"
#include "./joint_config.hpp"
#include "./rotor_config.hpp"
#include "./fixed_wing_config.hpp"
#include "./pwm_config.hpp"

template <>
struct rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>
{
  using is_specialized = std::true_type;
  using custom_type = tobas::Drone;
  using ros_message_type = tobas_drone_msgs::msg::Drone;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dst)
  {
    dst.name = src.name;

    tobas_drone_msgs::BatteryConfigAdapter::convert_to_ros_message(src.battery, dst.battery);

    dst.joints.clear();
    for (const auto& [_, joint] : src.joints)
    {
      dst.joints.emplace_back();
      tobas_drone_msgs::JointConfigAdapter::convert_to_ros_message(joint, dst.joints.back());
    }

    dst.rotors.clear();
    for (const auto& [_, rotor] : src.rotors)
    {
      dst.rotors.emplace_back();
      tobas_drone_msgs::RotorConfigAdapter::convert_to_ros_message(rotor, dst.rotors.back());
    }

    dst.pwms.clear();
    for (const auto& [_, pwm] : src.pwms)
    {
      dst.pwms.emplace_back();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_ros_message(pwm, dst.pwms.back());
    }

    tobas_drone_msgs::FixedWingConfigAdapter::convert_to_ros_message(src.fixed_wing, dst.fixed_wing);
  }

  static void convert_to_custom(const ros_message_type& src, custom_type& dst)
  {
    dst.name = src.name;

    tobas_drone_msgs::BatteryConfigAdapter::convert_to_custom(src.battery, dst.battery);

    dst.joints.clear();
    for (const auto& joint : src.joints)
    {
      dst.joints[joint.name] = tobas::JointConfig();
      tobas_drone_msgs::JointConfigAdapter::convert_to_custom(joint, dst.joints.at(joint.name));
    }

    dst.rotors.clear();
    for (const auto& rotor : src.rotors)
    {
      dst.rotors[rotor.channel] = tobas::RotorConfig();
      tobas_drone_msgs::RotorConfigAdapter::convert_to_custom(rotor, dst.rotors.at(rotor.channel));
    }

    dst.pwms.clear();
    for (const auto& pwm : src.pwms)
    {
      dst.pwms[pwm.joint_name] = tobas::PwmConfig();
      tobas_drone_msgs::PwmConfigAdapter::convert_to_custom(pwm, dst.pwms.at(pwm.joint_name));
    }

    tobas_drone_msgs::FixedWingConfigAdapter::convert_to_custom(src.fixed_wing, dst.fixed_wing);
  }
};

namespace tobas_drone_msgs
{
using DroneAdapter = rclcpp::TypeAdapter<tobas::Drone, tobas_drone_msgs::msg::Drone>;
}

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(tobas::Drone, tobas_drone_msgs::msg::Drone);
