// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs/msg/battery.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class FakeBattPublisherNode : public BaseNode
{
  using self = FakeBattPublisherNode;
  using super = BaseNode;

public:
  explicit FakeBattPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  double voltage_;  // [V]
  double current_;  // [A]

  ros2::PublisherPtr<tobas_msgs::msg::Battery> batt_pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakeBattPublisherNode::FakeBattPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_batt_publisher", nodeOptions_Default(options))
{
  voltage_ = getDoubleParam("voltage", 14.8);
  current_ = getDoubleParam("current", 20.0);

  batt_pub_ = createPublisher<tobas_msgs::msg::Battery>(topic::kBattery);
  timer_ = createTimer(10ms, &self::timerCb, this);
}

void FakeBattPublisherNode::timerCb()
{
  auto batt_msg = std::make_unique<tobas_msgs::msg::Battery>();
  batt_msg->header.stamp = now();
  batt_msg->voltage = voltage_;
  batt_msg->current = current_;

  batt_pub_->publish(std::move(batt_msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FakeBattPublisherNode)
