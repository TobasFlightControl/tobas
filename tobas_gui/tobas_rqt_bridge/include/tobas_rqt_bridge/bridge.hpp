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
namespace gui
{
class RosQtBridge : public QObject
{
  Q_OBJECT

  using self = RosQtBridge;
  using super = QObject;

Q_SIGNALS:
  void messageReceived(const tobas_msgs::msg::Message::ConstSharedPtr& msg);
  void batteryReceived(const tobas_msgs::msg::Battery::ConstSharedPtr& msg);
  void engineStateReceived(const tobas_msgs::msg::EngineState::ConstSharedPtr& msg);
  void cpuReceived(const tobas_msgs::msg::Cpu::ConstSharedPtr& msg);
  void sbusReceived(const tobas_msgs::msg::Sbus::ConstSharedPtr& msg);
  void rcInputReceived(const tobas_msgs::RCInput::ConstSharedPtr& msg);
  void imuReceived(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void magReceived(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void airPressureReceived(const tobas_msgs::msg::FluidPressure::ConstSharedPtr& msg);
  void gnssReceived(const tobas_msgs::Gnss::ConstSharedPtr& msg);
  void rotorStatesReceived(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessReceived(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);
  void jointStatesReceived(const tobas_msgs::msg::JointStateArray::ConstSharedPtr& msg);
  void odomReceived(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& msg);
  void armingReceived(const tobas_msgs::msg::Arming::ConstSharedPtr& msg);
  void vehicleHealthReceived(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& msg);
  void rosbagStateReceived(const tobas_msgs::msg::RosbagState::ConstSharedPtr& msg);
  void localHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void remoteHeartbeatReceived(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& msg);
  void rawImuReceived(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void rawMagReceived(const tobas_msgs::MagneticField::ConstSharedPtr& msg);

public:
  explicit RosQtBridge(QObject* parent = nullptr);

  void initializeRosInterfaces(const rclcpp::Node::SharedPtr& node, const std::string& ns);
  void clearRosInterfaces();

private:
  std::unordered_map<const char*, rclcpp::SubscriptionBase::SharedPtr> subs_;

  template <typename MsgType, auto SignalType>
  void add(
    const rclcpp::Node::SharedPtr& node,
    const std::string& ns,
    const char* base_topic,
    bool latch = ros2::qos::kDefaultLatch,
    bool reliable = ros2::qos::kDefaultReliable,
    size_t queue_size = ros2::qos::kDefaultQueueSize);
};
}  // namespace gui
}  // namespace tobas
