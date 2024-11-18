#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/magnetic_field_stamped.hpp>

#include "./base.hpp"

using HalMagThrottleNode = TopicThrottleNode<tobas_msgs::MagneticFieldStamped, real::kMagTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalMagThrottleNode)
