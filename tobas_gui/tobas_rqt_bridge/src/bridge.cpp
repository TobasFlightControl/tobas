// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rqt_bridge/bridge.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_real_common/ros_interface.hpp>

namespace tobas::rqt
{
RosQtBridge::RosQtBridge(QObject* _parent) : super(_parent)
{
}

void RosQtBridge::initializeRosInterfaces(const rclcpp::Node::SharedPtr& _node, const std::string& _ns)
{
  add<tobas_msgs::msg::Message, &self::messageReceived>(_node, _ns, topic::kMessage, false, true, 100);
  add<tobas_msgs::msg::Battery, &self::batteryReceived>(_node, _ns, topic::kBattery);
  add<tobas_msgs::msg::EngineState, &self::engineStateReceived>(_node, _ns, topic::kEngineState);
  add<tobas_msgs::msg::Cpu, &self::cpuReceived>(_node, _ns, topic::kCpu);
  add<tobas_msgs::msg::Sbus, &self::sbusReceived>(_node, _ns, topic::kSbus);
  add<tobas_msgs::RCInput, &self::rcInputReceived>(_node, _ns, topic::kRcInput);
  add<tobas_msgs::Imu, &self::imuReceived>(_node, _ns, topic::kImuFilt);
  add<tobas_msgs::MagneticField, &self::magReceived>(_node, _ns, topic::kMagneticField);
  add<tobas_msgs::msg::FluidPressure, &self::airPressureReceived>(_node, _ns, topic::kAirPressure);
  add<tobas_msgs::Gnss, &self::gnssReceived>(_node, _ns, topic::kGnss);
  add<tobas_msgs::msg::RotorStateArray, &self::rotorStatesReceived>(_node, _ns, topic::kRotorStates);
  add<tobas_msgs::msg::RotorLivelinessArray, &self::rotorLivelinessReceived>(_node, _ns, topic::kRotorLiv);
  add<tobas_msgs::msg::JointStateArray, &self::jointStatesReceived>(_node, _ns, topic::kJointStates);
  add<tobas_msgs::OdometryWithCovarianceStamped, &self::odomReceived>(_node, _ns, topic::kOdometry);
  add<tobas_msgs::msg::Arming, &self::armingReceived>(_node, _ns, topic::kArming);
  add<tobas_msgs::msg::VehicleHealth, &self::vehicleHealthReceived>(_node, _ns, topic::kVehicleHealth);
  add<tobas_msgs::msg::RosbagState, &self::rosbagStateReceived>(_node, _ns, topic::kRosbagState);
  add<tobas_msgs::msg::Heartbeat, &self::remoteHeartbeatReceived>(_node, _ns, topic::kHeartbeat);
  add<tobas_msgs::Imu, &self::rawImuReceived>(_node, _ns, real::topic::kImuRaw);
  add<tobas_msgs::MagneticField, &self::rawMagReceived>(_node, _ns, real::topic::kMagneticField);
}

void RosQtBridge::clearRosInterfaces()
{
  subs_.clear();
}

template <typename MsgType, auto SignalType>
void RosQtBridge::add(
  const rclcpp::Node::SharedPtr& _node,
  const std::string& _ns,
  const char* _base_topic,
  bool _latch,
  bool _reliable,
  size_t _queue_size)
{
  const auto topic = path::join(_ns, kRemoteIfaceNS, _base_topic);
  const ros2::qos::QoS qos(_latch, _reliable, _queue_size);
  const auto cb = [this](const typename MsgType::ConstSharedPtr& _msg) { (this->*SignalType)(_msg); };
  subs_[_base_topic] = _node->create_subscription<MsgType>(topic, qos, cb);
}
}  // namespace tobas::rqt
