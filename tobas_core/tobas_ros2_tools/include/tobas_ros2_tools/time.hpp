#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

namespace ros2
{
inline std::chrono::steady_clock::time_point chronoFromRosTime(const rclcpp::Time& ros_time)
{
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ros_time.nanoseconds()));
}
}  // namespace ros2
