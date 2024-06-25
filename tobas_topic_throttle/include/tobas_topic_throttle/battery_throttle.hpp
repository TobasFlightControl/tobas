#pragma once

#include <tobas_tools/constants.hpp>
#include <tobas_msgs/Battery.h>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using BatteryThrottle = TopicThrottle<tobas_msgs::Battery, tobas::kBatteryTopic>;
}
