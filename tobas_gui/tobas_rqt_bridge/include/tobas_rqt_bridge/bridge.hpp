// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <unordered_map>

#include <QObject>
#include <rclcpp/node.hpp>
#include <rclcpp/subscription_base.hpp>

#include <tobas_ros2_tools/qos.hpp>

#include "./declare.hpp"

namespace tobas
{
namespace rqt
{
class RosQtBridge : public QObject
{
  Q_OBJECT

  using self = RosQtBridge;
  using super = QObject;

Q_SIGNALS:
  void messageReceived(const tobas_msgs::msg::Message::ConstSharedPtr& _msg);
  void batteryReceived(const tobas_msgs::msg::Battery::ConstSharedPtr& _msg);
  void engineStateReceived(const tobas_msgs::msg::EngineState::ConstSharedPtr& _msg);
  void cpuReceived(const tobas_msgs::msg::Cpu::ConstSharedPtr& _msg);
  void sbusReceived(const tobas_msgs::msg::Sbus::ConstSharedPtr& _msg);
  void rcInputReceived(const tobas_msgs::RCInput::ConstSharedPtr& _msg);
  void imuReceived(const tobas_msgs::Imu::ConstSharedPtr& _msg);
  void magReceived(const tobas_msgs::MagneticField::ConstSharedPtr& _msg);
  void airPressureReceived(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& _msg);
  void gnssReceived(const tobas_msgs::Gnss::ConstSharedPtr& _msg);
  void rotorStatesReceived(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& _msg);
  void rotorLivelinessReceived(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& _msg);
  void jointStatesReceived(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& _msg);
  void odomReceived(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _msg);
  void armingReceived(const tobas_msgs::msg::Arming::ConstSharedPtr& _msg);
  void vehicleHealthReceived(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& _msg);
  void rosbagStateReceived(const tobas_msgs::msg::RosbagState::ConstSharedPtr& _msg);
  void localHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& _msg);
  void remoteHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& _msg);
  void rawImuReceived(const tobas_msgs::Imu::ConstSharedPtr& _msg);
  void rawMagReceived(const tobas_msgs::MagneticField::ConstSharedPtr& _msg);

public:
  explicit RosQtBridge(QObject* _parent = nullptr);

  void initializeRosInterfaces(const rclcpp::Node::SharedPtr& _node, const std::string& _ns);
  void clearRosInterfaces();

private:
  std::unordered_map<const char*, rclcpp::SubscriptionBase::SharedPtr> subs_;

  template <typename MsgType, auto SignalType>
  void add(
    const rclcpp::Node::SharedPtr& _node,
    const std::string& _ns,
    const char* _base_topic,
    bool _latch = ros2::qos::kDefaultLatch,
    bool _reliable = ros2::qos::kDefaultReliable,
    size_t _queue_size = ros2::qos::kDefaultQueueSize);
};
}  // namespace rqt
}  // namespace tobas
