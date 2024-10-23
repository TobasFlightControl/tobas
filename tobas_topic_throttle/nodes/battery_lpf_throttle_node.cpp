#include <tobas_msgs/msg/battery.hpp>

#include "./base.hpp"

using BatteryLPFThrottleNode = TopicThrottleNode<tobas_msgs::msg::Battery, tobas::kBatteryLpfTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryLPFThrottleNode)
