#pragma once

#include <rclcpp/time.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>

namespace ros2
{
inline long nanoseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<long>(stamp.sec) * 1'000'000'000L + static_cast<long>(stamp.nanosec);
}

inline double microseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-3;
}

inline double milliseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-6;
}

inline double seconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<double>(nanoseconds(stamp)) * 1e-9;
}

inline long nanoseconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<long>(duration.sec) * 1'000'000'000L + static_cast<long>(duration.nanosec);
}

inline double microseconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(nanoseconds(duration)) * 1e-3;
}

inline double milliseconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(nanoseconds(duration)) * 1e-6;
}

inline double seconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<double>(nanoseconds(duration)) * 1e-9;
}

inline std::chrono::steady_clock::time_point chronoFromRosTime(const rclcpp::Time& ros_time)
{
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ros_time.nanoseconds()));
}

inline std::chrono::steady_clock::time_point chronoFromRosTime(const builtin_interfaces::msg::Time& ros_time)
{
  const auto duration = std::chrono::seconds(ros_time.sec) + std::chrono::nanoseconds(ros_time.nanosec);
  return std::chrono::steady_clock::time_point(duration);
}

void timeChronoToMsg(const std::chrono::steady_clock::duration& c, builtin_interfaces::msg::Time& m);
}  // namespace ros2

rclcpp::Duration operator-(const builtin_interfaces::msg::Time& lhs, const builtin_interfaces::msg::Time& rhs);
