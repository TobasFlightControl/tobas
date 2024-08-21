#include <tobas_msgs/msg/latency.hpp>

#include "./base.hpp"

using LatencyThrottleNode = TopicThrottleNode<tobas_msgs::msg::Latency, tobas::kLatencyTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(LatencyThrottleNode)
