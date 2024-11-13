#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/sbus.hpp>

#include "./base.hpp"

using HalSbusThrottleNode = TopicThrottleNode<tobas_hal_msgs::msg::Sbus, hal::kSBUSTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalSbusThrottleNode)
