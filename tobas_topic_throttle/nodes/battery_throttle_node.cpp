#include <tobas_msgs/msg/battery.hpp>

#include "./base.hpp"

using BatteryThrottleNode = TopicThrottleNode<tobas_msgs::msg::Battery, tobas::kBatteryTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(BatteryThrottleNode)
