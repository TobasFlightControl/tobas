#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_tools/node.hpp>
#include <tobas_msgs/ThrottleArray.h>

namespace tobas_latency_publisher
{
class LatencyPublisher : public tobas::BaseNode
{
  using self = LatencyPublisher;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisher(
    rclcpp::Node::SharedPtr node,
    rclcpp::Node::SharedPtr pnh,
    const std::string& name = rclcpp::this_node::getName());

private:
  rclcpp::Publisher latency_pub_;
  rclcpp::Subscriber throttles_sub_;

  void throttlesCb(const tobas_msgs::ThrottleArrayConstPtr& msg);
};
}  // namespace tobas_latency_publisher
