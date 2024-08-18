#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/rc_input.hpp>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using RCInputThrottle = TopicThrottle<tobas_msgs::msg::RCInput, tobas::kRcInputTopic>;
}
