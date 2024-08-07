#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using BatteryLPFThrottle = TopicThrottle<tobas_msgs::Battery, tobas::kBatteryLpfTopic>;
}
