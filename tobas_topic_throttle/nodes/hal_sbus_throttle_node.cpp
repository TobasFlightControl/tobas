#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/sbus.hpp>

#include "./base.hpp"

using HalSbusThrottleNode = TopicThrottleNode<tobas_msgs::msg::Sbus, real::kSBUSTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalSbusThrottleNode)
