#pragma once

#include <tobas_tools/constants.hpp>
#include <tobas_kdl_msgs/EulerStamped.h>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using EulerThrottle = TopicThrottle<tobas_kdl_msgs::EulerStamped, tobas::kEulerTopic>;
}
