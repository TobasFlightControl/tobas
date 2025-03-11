#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/latency.hpp>

namespace tobas
{
class ControlLatencyPublisher
{
public:
  explicit ControlLatencyPublisher();

  void initialize(rclcpp::Node::SharedPtr node);
  void publish(const builtin_interfaces::msg::Time& start_time);

private:
  rclcpp::Node::SharedPtr node_;
  ros2::PublisherPtr<tobas_msgs::msg::Latency> latency_pub_;
};
}  // namespace tobas
