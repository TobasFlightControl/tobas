#include <tobas_ros2_tools/time.hpp>
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
  pub_ = ros2::createPublisher<tobas_msgs::msg::Latency>(node, tobas::kControlLatencyTopic);
}

void ControlLatencyPublisher::publish(const builtin_interfaces::msg::Time& start_time)
{
  const builtin_interfaces::msg::Time cur_time(node_->get_clock()->now());  // クロックタイプの削除して例外を回避

  auto msg = std::make_unique<tobas_msgs::msg::Latency>();
  msg->header.stamp = cur_time;
  msg->data = cur_time - start_time;

  pub_->publish(std::move(msg));
}
}  // namespace tobas
