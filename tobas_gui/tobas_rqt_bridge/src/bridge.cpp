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
RosQtBridge::RosQtBridge(rclcpp::Node::SharedPtr node) : node_(node)
{
  addGlobal<tobas_msgs::msg::Heartbeat, &self::localHeartbeatReceived>(topic::kHeartbeat);
}

void RosQtBridge::initializeScopedTopics(const std::string& ns)
{
  static constexpr auto ri = kRemoteIfaceNS;

  scoped_subs_.clear();

  addScoped<tobas_msgs::msg::Message, &self::messageReceived>(ns, path::join(ri, topic::kMessage), false, true, 100);
  addScoped<tobas_msgs::msg::Battery, &self::batteryReceived>(ns, path::join(ri, topic::kBattery));
  addScoped<tobas_msgs::msg::EngineState, &self::engineStateReceived>(ns, path::join(ri, topic::kEngineState));
  addScoped<tobas_msgs::msg::Cpu, &self::cpuReceived>(ns, path::join(ri, topic::kCpu));
  addScoped<tobas_msgs::msg::Sbus, &self::sbusReceived>(ns, path::join(ri, topic::kSbus));
  addScoped<tobas_msgs::RCInput, &self::rcInputReceived>(ns, path::join(ri, topic::kRcInput));
  addScoped<tobas_msgs::Imu, &self::imuReceived>(ns, path::join(ri, topic::kImuFilt));
  addScoped<tobas_msgs::MagneticField, &self::magReceived>(ns, path::join(ri, topic::kMagneticField));
  addScoped<tobas_msgs::msg::FluidPressure, &self::airPressureReceived>(ns, path::join(ri, topic::kAirPressure));
  addScoped<tobas_msgs::Gnss, &self::gnssReceived>(ns, path::join(ri, topic::kGnss));
  addScoped<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(ns, path::join(ri, topic::kRotorStates));
  addScoped<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessReceived>(ns, path::join(ri, topic::kRotorLiv));
  addScoped<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(ns, path::join(ri, topic::kJointStates));
  addScoped<tobas_msgs::OdometryWithCovarianceStamped, &self::odomReceived>(ns, path::join(ri, topic::kOdometry));
  addScoped<tobas_msgs::msg::Arming, &self::armingReceived>(ns, path::join(ri, topic::kArming));
  addScoped<tobas_msgs::msg::VehicleHealth, &self::vehicleHealthReceived>(ns, path::join(ri, topic::kVehicleHealth));
  addScoped<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(ns, path::join(ri, topic::kRosbagState));
  addScoped<tobas_msgs::msg::Heartbeat, &self::remoteHeartbeatReceived>(ns, topic::kHeartbeat);
  addScoped<tobas_msgs::Imu, &self::rawImuReceived>(ns, path::join(ri, real::topic::kImuRaw));
  addScoped<tobas_msgs::MagneticField, &self::rawMagReceived>(ns, path::join(ri, real::topic::kMagneticField));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(
  const std::string& topic,
  std::vector<rclcpp::SubscriptionBase::SharedPtr>& buf,
  bool latch,
  bool reliable,
  size_t queue_size)
{
  const ros2::qos::QoS qos(latch, reliable, queue_size);
  const auto cb = [this](const typename MsgType::ConstSharedPtr& msg) { (this->*SignalType)(msg); };
  buf.push_back(node_->create_subscription<MsgType>(topic, qos, cb));
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addGlobal(const std::string& topic, bool latch, bool reliable, size_t queue_size)
{
  add<MsgType, SignalType>(topic, global_subs_, latch, reliable, queue_size);
}

template <typename MsgType, auto SignalType>
void RosQtBridge::addScoped(const std::string& ns, const std::string& topic, bool latch, bool reliable, size_t queue_size)
{
  add<MsgType, SignalType>(path::join(ns, topic), scoped_subs_, latch, reliable, queue_size);
}
}  // namespace gui
}  // namespace tobas
