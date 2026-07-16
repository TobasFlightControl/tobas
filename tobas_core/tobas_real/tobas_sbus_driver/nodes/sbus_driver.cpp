// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/sbus.hpp>

#include "tobas_sbus_driver/sbus.hpp"

using namespace std::chrono_literals;

namespace tobas
{
class SbusDriverNode : public BaseNode
{
  using self = SbusDriverNode;
  using super = BaseNode;

public:
  explicit SbusDriverNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  std::string device_;
  SBUS sbus_;
  std::mutex mutex_;

  ros2::PublisherPtr<tobas_msgs::msg::Sbus> sbus_pub_;
  ros2::TimerPtr initialize_timer_;
  ros2::TimerPtr timeout_timer_;

  void initialize();

  void publishExclusively(tobas_msgs::msg::Sbus::UniquePtr msg);

  void onPacket(const SBUS::Packet& packet);

  void onPacketTimeout();
};

SbusDriverNode::SbusDriverNode(const rclcpp::NodeOptions& options)
  : super("sbus_driver", nodeOptions_Default(options)), sbus_(std::bind(&self::onPacket, this, std::placeholders::_1))
{
  device_ = getStringParam("device", "");
  if (device_.empty()) {
    TOBAS_WARN("No device name specified. This node will not work.");
    return;
  }

  sbus_pub_ = createPublisher<tobas_msgs::msg::Sbus>(topic::kSbus);

  initialize_timer_ = createWallTimer(3s, &self::initialize, this);
}

void SbusDriverNode::initialize()
{
  if (!sbus_.initialize(device_.c_str())) {
    TOBAS_WARN("Failed to initialize the S.BUS driver with device \"", device_, "\". Retrying...");
    return;
  }

  initialize_timer_->cancel();

  sbus_.start();

  // The longest S.BUS period is 14 ms.
  // Treat the receiver as abnormal if 3 or more frames are missing.
  // Always publish some message because subscribers are harder to implement as event-driven logic if messages may stop.
  timeout_timer_ = createWallTimer(14ms * 3, &self::onPacketTimeout, this);
}

void SbusDriverNode::publishExclusively(tobas_msgs::msg::Sbus::UniquePtr msg)
{
  // Lock to prevent the packet reception and timeout threads from accessing the publisher at the same time.
  const std::lock_guard lock(mutex_);
  sbus_pub_->publish(std::move(msg));
}

void SbusDriverNode::onPacket(const SBUS::Packet& packet)
{
  // Reset the timeout timer.
  timeout_timer_.reset();

  // Create a message.
  auto sbus_msg = std::make_unique<tobas_msgs::msg::Sbus>();
  sbus_msg->header.stamp = now();

  sbus_msg->periods = packet.periods;
  sbus_msg->ch17 = packet.ch17;
  sbus_msg->ch18 = packet.ch18;
  sbus_msg->frame_lost = packet.frame_lost;
  sbus_msg->failsafe = packet.failsafe;

  // Publish the message.
  publishExclusively(std::move(sbus_msg));
}

void SbusDriverNode::onPacketTimeout()
{
  // Publish a frame-lost message.
  auto sbus_msg = std::make_unique<tobas_msgs::msg::Sbus>();
  sbus_msg->header.stamp = now();
  sbus_msg->frame_lost = true;
  publishExclusively(std::move(sbus_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::SbusDriverNode)
