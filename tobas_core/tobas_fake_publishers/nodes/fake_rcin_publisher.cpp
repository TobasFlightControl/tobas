// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <tobas_constants/ros_interface.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/rc_input.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class FakeRcInputPublisherNode : public BaseNode
{
  static constexpr auto kSamplingPeriod = 10ms;

  using self = FakeRcInputPublisherNode;
  using super = BaseNode;

public:
  explicit FakeRcInputPublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  ros2::PublisherPtr<tobas_msgs::RCInput> pub_;
  ros2::TimerPtr timer_;

  void timerCb();
};

FakeRcInputPublisherNode::FakeRcInputPublisherNode(const rclcpp::NodeOptions& options)
  : super("fake_rcin_publisher", nodeOptions_Default(options))
{
  pub_ = createPublisher<tobas_msgs::RCInput>(topic::kRcInput);
  timer_ = createTimer(kSamplingPeriod, &self::timerCb, this);
}

void FakeRcInputPublisherNode::timerCb()
{
  auto msg = std::make_unique<tobas_msgs::RCInput>();
  msg->header.stamp = now();
  msg->ok = true;
  msg->roll = 0.0;
  msg->pitch = 0.0;
  msg->throttle = -1.0;
  msg->yaw = 0.0;
  msg->mode = FlightMode::kAcrobat;
  msg->sub_mode = false;
  msg->enable = false;
  msg->kill = false;
  msg->gpsw.fill(false);

  pub_->publish(std::move(msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::FakeRcInputPublisherNode)
