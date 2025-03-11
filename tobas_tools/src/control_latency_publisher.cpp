#include <tobas_constants/constants.hpp>

#include "../include/tobas_tools/control_latency_publisher.hpp"

namespace tobas
{
ControlLatencyPublisher::ControlLatencyPublisher()
{
}

void ControlLatencyPublisher::initialize(rclcpp::Node::SharedPtr node)
{
  node_ = node;
  latency_pub_ = ros2::createPublisher<tobas_msgs::msg::Latency>(node, tobas::kControlLatencyTopic);
}

void ControlLatencyPublisher::publish(const builtin_interfaces::msg::Time& start_time)
{
  const auto cur_time = node_->get_clock()->now();

  auto latency = std::make_unique<tobas_msgs::msg::Latency>();
  latency->header.stamp = cur_time;
  latency->data = cur_time - start_time;

  latency_pub_->publish(std::move(latency));
}
}  // namespace tobas
