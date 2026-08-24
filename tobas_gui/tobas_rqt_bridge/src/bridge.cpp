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
  global_subs_.clear();
  scoped_subs_.clear();

  addGlobal<tobas_msgs::msg::Heartbeat, &self::localHeartbeatReceived>(node, topic::kHeartbeat);
  static constexpr auto ri = kRemoteIfaceNS;

  addScoped<tobas_msgs::msg::Message, &self::messageReceived>(
    node, ns, path::join(ri, topic::kMessage), false, true, 100);
  addScoped<tobas_msgs::msg::Battery, &self::batteryReceived>(node, ns, path::join(ri, topic::kBattery));
  addScoped<tobas_msgs::msg::EngineState, &self::engineStateReceived>(node, ns, path::join(ri, topic::kEngineState));
  addScoped<tobas_msgs::msg::Cpu, &self::cpuReceived>(node, ns, path::join(ri, topic::kCpu));
  addScoped<tobas_msgs::msg::Sbus, &self::sbusReceived>(node, ns, path::join(ri, topic::kSbus));
  addScoped<tobas_msgs::RCInput, &self::rcInputReceived>(node, ns, path::join(ri, topic::kRcInput));
  addScoped<tobas_msgs::Imu, &self::imuReceived>(node, ns, path::join(ri, topic::kImuFilt));
  addScoped<tobas_msgs::MagneticField, &self::magReceived>(node, ns, path::join(ri, topic::kMagneticField));
  addScoped<tobas_msgs::msg::FluidPressure, &self::airPressureReceived>(node, ns, path::join(ri, topic::kAirPressure));
  addScoped<tobas_msgs::Gnss, &self::gnssReceived>(node, ns, path::join(ri, topic::kGnss));
  addScoped<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(node, ns, path::join(ri, topic::kRotorStates));
  addScoped<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessReceived>(
    node, ns, path::join(ri, topic::kRotorLiv));
  addScoped<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(node, ns, path::join(ri, topic::kJointStates));
  addScoped<tobas_msgs::OdometryWithCovarianceStamped, &self::odomReceived>(node, ns, path::join(ri, topic::kOdometry));
  addScoped<tobas_msgs::msg::Arming, &self::armingReceived>(node, ns, path::join(ri, topic::kArming));
  addScoped<tobas_msgs::msg::VehicleHealth, &self::vehicleHealthReceived>(
    node, ns, path::join(ri, topic::kVehicleHealth));
  addScoped<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(node, ns, path::join(ri, topic::kRosbagState));
  addScoped<tobas_msgs::msg::Heartbeat, &self::remoteHeartbeatReceived>(node, ns, path::join(ri, topic::kHeartbeat));
  addScoped<tobas_msgs::Imu, &self::rawImuReceived>(node, ns, path::join(ri, real::topic::kImuRaw));
  addScoped<tobas_msgs::MagneticField, &self::rawMagReceived>(node, ns, path::join(ri, real::topic::kMagneticField));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(
  const rclcpp::Node::SharedPtr& node,
  const std::string& topic,
  std::vector<rclcpp::SubscriptionBase::SharedPtr>& buf,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this](const typename MsgType::ConstSharedPtr& msg) { (this->*SignalType)(msg); };
  buf.push_back(node->create_subscription<MsgType>(topic, qos, cb));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addGlobal(
  const rclcpp::Node::SharedPtr& node,
  const std::string& topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  add<MsgType, SignalType>(node, topic, global_subs_, latch, reliable, queue_size);
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addScoped(
  const rclcpp::Node::SharedPtr& node,
  const std::string& ns,
  const std::string& topic,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  add<MsgType, SignalType>(node, path::join(ns, topic), scoped_subs_, latch, reliable, queue_size);
}
}  // namespace gui
}  // namespace tobas
