#include <tobas_msgs/msg/rotor_speeds.hpp>

#include "./base.hpp"

using RotorSpeedsThrottleNode = TopicThrottleNode<tobas_msgs::msg::RotorSpeeds, tobas::kRotorSpeedsTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(RotorSpeedsThrottleNode)
