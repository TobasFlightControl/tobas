// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_real_common/ros_interface.hpp>

#include <tobas_drone_msgs/msg/drone.hpp>
#include <tobas_kdl_msgs/msg/tree.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/battery.hpp>
#include <tobas_msgs/msg/cpu.hpp>
#include <tobas_msgs/msg/engine_state.hpp>
#include <tobas_msgs/msg/fluid_pressure.hpp>
#include <tobas_msgs/msg/gnss.hpp>
#include <tobas_msgs/msg/imu.hpp>
#include <tobas_msgs/msg/joint_state_array.hpp>
#include <tobas_msgs/msg/magnetic_field.hpp>
#include <tobas_msgs/msg/message.hpp>
#include <tobas_msgs/msg/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/rosbag_state.hpp>
#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>
#include <tobas_msgs/msg/sbus.hpp>
#include <tobas_msgs/msg/vehicle_health.hpp>

#include "./ros_interface.hpp"

namespace tobas
{
void RosInterfaceNode::registerTopicsLogicToIface()
{
  addTopicLogicToIface<tobas_msgs::msg::Message>(topic::kMessage, topic::kMessage, false, true, 100);
  addTopicLogicToIface<tobas_drone_msgs::msg::Drone>(topic::kDrone, topic::kDrone, true, true);
  addTopicLogicToIface<tobas_kdl_msgs::msg::Tree>(topic::kKdlTree, topic::kKdlTree, true, true);
  addTopicLogicToIface<tobas_msgs::msg::Battery>(addThrotNS(topic::kBattery), topic::kBattery);
  addTopicLogicToIface<tobas_msgs::msg::EngineState>(addThrotNS(topic::kEngineState), topic::kEngineState);
  addTopicLogicToIface<tobas_msgs::msg::Cpu>(topic::kCpu, topic::kCpu);
  addTopicLogicToIface<tobas_msgs::msg::Sbus>(addThrotNS(topic::kSbus), topic::kSbus);
  addTopicLogicToIface<tobas_msgs::msg::RCInput>(addThrotNS(topic::kRcInput), topic::kRcInput);
  addTopicLogicToIface<tobas_msgs::msg::Imu>(addThrotNS(topic::kImuFilt), topic::kImuFilt);
  addTopicLogicToIface<tobas_msgs::msg::MagneticField>(addThrotNS(topic::kMagneticField), topic::kMagneticField);
  addTopicLogicToIface<tobas_msgs::msg::FluidPressure>(addThrotNS(topic::kAirPressure), topic::kAirPressure);
  addTopicLogicToIface<tobas_msgs::msg::Gnss>(topic::kGnss, topic::kGnss);
  addTopicLogicToIface<tobas_msgs::msg::RotorStateArray>(addThrotNS(topic::kRotorStates), topic::kRotorStates);
  addTopicLogicToIface<tobas_msgs::msg::RotorLivelinessArray>(topic::kRotorLiv, topic::kRotorLiv);
  addTopicLogicToIface<tobas_msgs::msg::JointStateArray>(addThrotNS(topic::kJointStates), topic::kJointStates);
  addTopicLogicToIface<tobas_msgs::msg::OdometryWithCovarianceStamped>(addThrotNS(topic::kOdometry), topic::kOdometry);
  addTopicLogicToIface<tobas_msgs::msg::Arming>(topic::kArming, topic::kArming);
  addTopicLogicToIface<tobas_msgs::msg::VehicleHealth>(topic::kVehicleHealth, topic::kVehicleHealth);
  addTopicLogicToIface<tobas_msgs::msg::Imu>(addThrotNS(real::topic::kImuRaw), real::topic::kImuRaw);
  addTopicLogicToIface<tobas_msgs::msg::MagneticField>(
    addThrotNS(real::topic::kMagneticField), real::topic::kMagneticField);
  addTopicLogicToIface<tobas_msgs::msg::RosbagState>(topic::kRosbagState, topic::kRosbagState);
}
}  // namespace tobas
