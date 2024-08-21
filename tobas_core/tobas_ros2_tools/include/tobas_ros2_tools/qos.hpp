#pragma once

#include <rclcpp/rclcpp.hpp>

namespace ros2
{
rclcpp::QoS makeQoS(bool latch, bool reliable, size_t queue_size);
}
