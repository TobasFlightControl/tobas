#pragma once

#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace tobas
{
namespace ros2
{
inline int64_t nanoseconds(const builtin_interfaces::msg::Time& stamp)
{
  return static_cast<int64_t>(stamp.sec) * 1'000'000'000L + static_cast<int64_t>(stamp.nanosec);
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

inline int64_t nanoseconds(const builtin_interfaces::msg::Duration& duration)
{
  return static_cast<int64_t>(duration.sec) * 1'000'000'000L + static_cast<int64_t>(duration.nanosec);
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
}  // namespace tobas

inline rclcpp::Duration operator-(const builtin_interfaces::msg::Time& lhs, const builtin_interfaces::msg::Time& rhs)
{
  const auto lhs_ns = RCL_S_TO_NS(lhs.sec) + lhs.nanosec;
  const auto rhs_ns = RCL_S_TO_NS(rhs.sec) + rhs.nanosec;
  const auto dur_ns = lhs_ns - rhs_ns;
  return rclcpp::Duration::from_nanoseconds(dur_ns);
}

inline bool operator>(const builtin_interfaces::msg::Duration& lhs, const std::chrono::nanoseconds& rhs)
{
  return tobas::ros2::nanoseconds(lhs) > rhs.count();
}
