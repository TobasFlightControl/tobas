// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/heartbeat.hpp>
#include <tobas_msgs/msg/remote_connection.hpp>

namespace ch = std::chrono;

namespace tobas
{
class HeartbeatReceiverNode : public BaseNode
{
  using self = HeartbeatReceiverNode;
  using super = BaseNode;

public:
  explicit HeartbeatReceiverNode(const rclcpp::NodeOptions& _options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::msg::RemoteConnection> connection_pub_;
  ros2::SubscriberPtr<tobas_msgs::msg::Heartbeat> heartbeat_sub_;
  ros2::TimerPtr timeout_timer_;

  void publishConnectionState(bool _connected);

  void heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr& _msg);
  void onConnectionTimeout();
};

HeartbeatReceiverNode::HeartbeatReceiverNode(const rclcpp::NodeOptions& _options)
  : super("heartbeat_receiver", nodeOptions_Default(_options))
{
  const auto timeout = getDoubleParam("timeout");  // [s]

  connection_pub_ = createPublisher<tobas_msgs::msg::RemoteConnection>(topic::kRemoteConnection);
  heartbeat_sub_ = createSubscriber(topic::kHeartbeat, &self::heartbeatCb, this);

  timeout_timer_ = createTimer(ch::duration<double>(timeout), &self::onConnectionTimeout, this);
}

void HeartbeatReceiverNode::publishConnectionState(bool _connected)
{
  auto msg = std::make_unique<tobas_msgs::msg::RemoteConnection>();
  msg->header.stamp = now();
  msg->data = _connected;
  connection_pub_->publish(std::move(msg));
}

void HeartbeatReceiverNode::heartbeatCb(const tobas_msgs::msg::Heartbeat::ConstSharedPtr&)
{
  timeout_timer_->reset();
  publishConnectionState(true);
}

void HeartbeatReceiverNode::onConnectionTimeout()
{
  publishConnectionState(false);
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::HeartbeatReceiverNode)
