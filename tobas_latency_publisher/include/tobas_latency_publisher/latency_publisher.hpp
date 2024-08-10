#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_node/node.hpp>
#include <tobas_msgs/msg/throttle_array.hpp>

namespace tobas_latency_publisher
{
class LatencyPublisher : public tobas::BaseNode
{
  using self = LatencyPublisher;
  using super = tobas::BaseNode;

public:
  explicit LatencyPublisher(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  PublisherPtr<> latency_pub_;
  SubscriberPtr<> throttles_sub_;

  void throttlesCb(const tobas_msgs::msg::ThrottleArray::ConstSharedPtr& msg);
};
}  // namespace tobas_latency_publisher
