#include <tobas_hal_core/constants.hpp>
#include <tobas_hal_msgs_adapter/Imu.hpp>

#include "./base.hpp"

using HalImuThrottleNode = TopicThrottleNode<tobas_hal_msgs::Imu, hal::kIMUTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(HalImuThrottleNode)
