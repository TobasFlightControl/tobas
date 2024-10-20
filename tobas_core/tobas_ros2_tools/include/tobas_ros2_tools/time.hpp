#pragma once

#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace ros2
{
inline std::chrono::steady_clock::time_point chronoFromRosTime(const rclcpp::Time& ros_time)
{
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ros_time.nanoseconds()));
}

void timeChronoToMsg(const std::chrono::steady_clock::duration& c, builtin_interfaces::msg::Time& m);
void timeMsgToChrono(const builtin_interfaces::msg::Time& m, std::chrono::steady_clock::duration& c);
}  // namespace ros2

rclcpp::Duration operator-(const builtin_interfaces::msg::Time& lhs, const builtin_interfaces::msg::Time& rhs);
