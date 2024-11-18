#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/MagneticFieldStamped.hpp>

#include "./base.hpp"

using HalMagThrottleNode = TopicThrottleNode<tobas_msgs::MagneticFieldStamped, real::kMagTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalMagThrottleNode)
