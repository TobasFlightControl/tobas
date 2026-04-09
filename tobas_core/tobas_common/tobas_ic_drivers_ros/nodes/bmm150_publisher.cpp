// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <tobas_ic_drivers/bmm150.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

namespace tobas
{
class Bmm150PublisherNode : public rclcpp::Node
{
public:
  explicit Bmm150PublisherNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  bool initialize();
  void timerCallback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
  driver::BMM150 mag_;
  double mx_, my_, mz_;  // [uT]
  bool initialized_ = false;
};

Bmm150PublisherNode::Bmm150PublisherNode(const rclcpp::NodeOptions& options) : Node("bmm150_publisher", options)
{
  publisher_ = create_publisher<geometry_msgs::msg::PointStamped>("magnetic_field", 1);
  timer_ = create_wall_timer(100ms, std::bind(&Bmm150PublisherNode::timerCallback, this));
}

bool Bmm150PublisherNode::initialize()
{
  if (!mag_.initialize()) {
    RCLCPP_WARN(get_logger(), "Failed to initialize magnetometer.");
    return false;
  }

  return true;
}

void Bmm150PublisherNode::timerCallback()
{
  if (!initialized_) {
    initialized_ = initialize();
    return;
  }

  if (!mag_.readMag(mx_, my_, mz_)) {
    RCLCPP_WARN(get_logger(), "Failed to read magnetic field.");
    return;
  }

  auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  msg->header.frame_id = "map";
  msg->point.x = mx_;
  msg->point.y = my_;
  msg->point.z = mz_;
  RCLCPP_INFO(get_logger(), "Publishing: '%lf, %lf, %lf' [μT]", mx_, my_, mz_);
  publisher_->publish(std::move(msg));
}
}  // namespace tobas

RCLCPP_COMPONENTS_REGISTER_NODE(tobas::Bmm150PublisherNode)
