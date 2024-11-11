#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./base.hpp"

using RotorStatesThrottleNode = TopicThrottleNode<tobas_msgs::msg::RotorStateArray, tobas::kRotorStatesTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(RotorStatesThrottleNode)
