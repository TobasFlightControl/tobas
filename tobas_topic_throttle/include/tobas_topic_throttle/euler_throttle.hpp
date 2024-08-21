#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_kdl_msgs_adapter/EulerStamped.hpp>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using EulerThrottle = TopicThrottle<tobas_kdl_msgs::EulerStamped, tobas::kEulerTopic>;
}
