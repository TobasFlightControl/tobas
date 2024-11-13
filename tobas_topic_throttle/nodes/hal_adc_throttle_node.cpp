#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs/msg/adc.hpp>

#include "./base.hpp"

using HalAdcThrottleNode = TopicThrottleNode<tobas_hal_msgs::msg::Adc, hal::kADCTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalAdcThrottleNode)
