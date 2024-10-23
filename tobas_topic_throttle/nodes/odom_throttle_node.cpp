#include <tobas_msgs_adapter/Odometry.hpp>

#include "./base.hpp"

using OdometryThrottleNode = TopicThrottleNode<tobas_msgs::Odometry, tobas::kOdometryTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(OdometryThrottleNode)
