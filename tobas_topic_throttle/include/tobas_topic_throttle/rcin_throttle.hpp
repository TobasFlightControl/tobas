#pragma once

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/RCInput.h>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using RCInputThrottle = TopicThrottle<tobas_msgs::RCInput, tobas::kRcInputTopic>;
}
