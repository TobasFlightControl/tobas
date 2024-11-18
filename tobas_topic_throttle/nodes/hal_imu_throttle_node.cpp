#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/ImuRaw.hpp>

#include "./base.hpp"

using HalImuThrottleNode = TopicThrottleNode<tobas_msgs::ImuRaw, real::kIMUTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalImuThrottleNode)
