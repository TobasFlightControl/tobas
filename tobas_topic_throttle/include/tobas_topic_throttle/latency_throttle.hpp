#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_msgs_adapter/Latency.h>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using LatencyThrottle = TopicThrottle<tobas_msgs::msg::Latency, tobas::kLatencyTopic>;
}
