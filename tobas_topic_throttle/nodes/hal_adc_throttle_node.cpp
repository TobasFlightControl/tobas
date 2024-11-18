#include <tobas_real_common/constants.hpp>
#include <tobas_msgs/msg/adc.hpp>

#include "./base.hpp"

using HalAdcThrottleNode = TopicThrottleNode<tobas_msgs::msg::Adc, real::kADCTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalAdcThrottleNode)
