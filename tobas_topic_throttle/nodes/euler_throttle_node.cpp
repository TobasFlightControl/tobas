#include <tobas_kdl_msgs_adapter/euler_stamped.hpp>

#include "./base.hpp"

using EulerThrottleNode = TopicThrottleNode<tobas_kdl_msgs::EulerStamped, tobas::kEulerTopic>;

RCLCPP_COMPONENTS_REGISTER_NODE(EulerThrottleNode)
