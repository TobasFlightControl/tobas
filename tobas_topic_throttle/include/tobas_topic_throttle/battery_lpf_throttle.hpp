#pragma once

#include <tobas_constants/constants.hpp>
#include <tobas_msgs/msg/battery.hpp>

#include "./base.hpp"

namespace tobas_topic_throttle
{
using BatteryLPFThrottle = TopicThrottle<tobas_msgs::msg::Battery, tobas::kBatteryLpfTopic>;
}
