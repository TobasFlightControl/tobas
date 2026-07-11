// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/heartbeat.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class HeartbeatSenderNode : public BaseNode
{
  using self = HeartbeatSenderNode;
  using super = BaseNode;

public:
  explicit HeartbeatSenderNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  uint64_t sequence_number_ = 0;
  ros2::PublisherPtr<tobas_msgs::msg::Heartbeat> heartbeat_pub_;
  ros2::TimerPtr main_timer_;

  void mainTimerCb();
};

HeartbeatSenderNode::HeartbeatSenderNode(const rclcpp::NodeOptions& options)
  : super("heartbeat_sender", nodeOptions_Default(options))
{
  heartbeat_pub_ = createPublisher<tobas_msgs::msg::Heartbeat>(topic::kHeartbeat);
  main_timer_ = createTimer(200ms, &self::mainTimerCb, this);
}

void HeartbeatSenderNode::mainTimerCb()
{
  auto heartbeat = std::make_unique<tobas_msgs::msg::Heartbeat>();
  heartbeat->header.stamp = now();
  heartbeat->sequence_number = sequence_number_++;
  heartbeat_pub_->publish(std::move(heartbeat));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::HeartbeatSenderNode)
