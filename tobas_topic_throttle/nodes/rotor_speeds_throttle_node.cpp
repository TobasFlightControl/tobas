#include <tobas_msgs/msg/rotor_speed_array.hpp>

#include "./base.hpp"

using RotorSpeedsThrottleNode = TopicThrottleNode<tobas_msgs::msg::RotorSpeedArray, tobas::kRotorSpeedsTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(RotorSpeedsThrottleNode)
