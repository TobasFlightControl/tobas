// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rqt_bridge/bridge.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/ros_interface.hpp>

namespace tobas
{
namespace gui
{
RosQtBridge::RosQtBridge(QObject* parent) : super(parent)
{
}

void RosQtBridge::initializeRosInterfaces(const rclcpp::Node::SharedPtr& node, const std::string& ns)
{
  add<tobas_msgs::msg::Message, &self::messageReceived>(node, ns, topic::kMessage, false, true, 100);
  add<tobas_msgs::msg::Battery, &self::batteryReceived>(node, ns, topic::kBattery);
  add<tobas_msgs::msg::EngineState, &self::engineStateReceived>(node, ns, topic::kEngineState);
  add<tobas_msgs::msg::Cpu, &self::cpuReceived>(node, ns, topic::kCpu);
  add<tobas_msgs::msg::Sbus, &self::sbusReceived>(node, ns, topic::kSbus);
  add<tobas_msgs::RCInput, &self::rcInputReceived>(node, ns, topic::kRcInput);
  add<tobas_msgs::Imu, &self::imuReceived>(node, ns, topic::kImuFilt);
  add<tobas_msgs::MagneticField, &self::magReceived>(node, ns, topic::kMagneticField);
  add<tobas_msgs::msg::FluidPressure, &self::airPressureReceived>(node, ns, topic::kAirPressure);
  add<tobas_msgs::Gnss, &self::gnssReceived>(node, ns, topic::kGnss);
  add<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(node, ns, topic::kRotorStates);
  add<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessReceived>(node, ns, topic::kRotorLiv);
  add<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(node, ns, topic::kJointStates);
  add<tobas_msgs::OdometryWithCovarianceStamped, &self::odomReceived>(node, ns, topic::kOdometry);
  add<tobas_msgs::msg::Arming, &self::armingReceived>(node, ns, topic::kArming);
  add<tobas_msgs::msg::VehicleHealth, &self::vehicleHealthReceived>(node, ns, topic::kVehicleHealth);
  add<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(node, ns, topic::kRosbagState);
  add<tobas_msgs::msg::Heartbeat, &self::remoteHeartbeatReceived>(node, ns, topic::kHeartbeat);
  add<tobas_msgs::Imu, &self::rawImuReceived>(node, ns, real::topic::kImuRaw);
  add<tobas_msgs::MagneticField, &self::rawMagReceived>(node, ns, real::topic::kMagneticField);
}

void RosQtBridge::clearRosInterfaces()
{
  subs_.clear();
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(
  const rclcpp::Node::SharedPtr& node,
  const std::string& ns,
  const char* base_topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const auto topic = path::join(ns, kRemoteIfaceNS, base_topic);
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this](const typename MsgType::ConstSharedPtr& msg) { (this->*SignalType)(msg); };
  subs_[base_topic] = node->create_subscription<MsgType>(topic, qos, cb);
}
}  // namespace gui
}  // namespace tobas
