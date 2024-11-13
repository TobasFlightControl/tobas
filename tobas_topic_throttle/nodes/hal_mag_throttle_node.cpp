#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/MagneticField.hpp>

#include "./base.hpp"

using HalMagThrottleNode = TopicThrottleNode<tobas_hal_msgs::MagneticField, hal::kMagTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalMagThrottleNode)
