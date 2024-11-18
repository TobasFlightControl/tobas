#include <tobas_real_common/constants.hpp>
#include <tobas_msgs_adapter/imu_stamped.hpp>

#include "./base.hpp"

using HalImuThrottleNode = TopicThrottleNode<tobas_msgs::ImuStamped, real::kIMUTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalImuThrottleNode)
