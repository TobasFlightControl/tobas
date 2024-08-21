#include <tobas_msgs/msg/rc_input.hpp>

#include "./base.hpp"

using RCInputThrottleNode = TopicThrottleNode<tobas_msgs::msg::RCInput, tobas::kRcInputTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(RCInputThrottleNode)
