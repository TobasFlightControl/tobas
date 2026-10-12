// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/time.hpp>

#include <builtin_interfaces/msg/duration.hpp>
#include <builtin_interfaces/msg/time.hpp>

namespace tobas
{
namespace ros2
{
inline int64_t nanoseconds(const builtin_interfaces::msg::Time& _stamp)
{
  return static_cast<int64_t>(_stamp.sec) * 1'000'000'000L + static_cast<int64_t>(_stamp.nanosec);
}

inline double microseconds(const builtin_interfaces::msg::Time& _stamp)
{
  return static_cast<double>(nanoseconds(_stamp)) * 1e-3;
}

inline double milliseconds(const builtin_interfaces::msg::Time& _stamp)
{
  return static_cast<double>(nanoseconds(_stamp)) * 1e-6;
}

inline double seconds(const builtin_interfaces::msg::Time& _stamp)
{
  return static_cast<double>(nanoseconds(_stamp)) * 1e-9;
}

inline int64_t nanoseconds(const builtin_interfaces::msg::Duration& _duration)
{
  return static_cast<int64_t>(_duration.sec) * 1'000'000'000L + static_cast<int64_t>(_duration.nanosec);
}

inline double microseconds(const builtin_interfaces::msg::Duration& _duration)
{
  return static_cast<double>(nanoseconds(_duration)) * 1e-3;
}

inline double milliseconds(const builtin_interfaces::msg::Duration& _duration)
{
  return static_cast<double>(nanoseconds(_duration)) * 1e-6;
}

inline double seconds(const builtin_interfaces::msg::Duration& _duration)
{
  return static_cast<double>(nanoseconds(_duration)) * 1e-9;
}

inline std::chrono::steady_clock::time_point chronoFromRosTime(const rclcpp::Time& _ros_time)
{
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(_ros_time.nanoseconds()));
}

inline std::chrono::steady_clock::time_point chronoFromRosTime(const builtin_interfaces::msg::Time& _ros_time)
{
  const auto duration = std::chrono::seconds(_ros_time.sec) + std::chrono::nanoseconds(_ros_time.nanosec);
  return std::chrono::steady_clock::time_point(duration);
}

void timeChronoToMsg(const std::chrono::steady_clock::duration& _c, builtin_interfaces::msg::Time& _m);
}  // namespace ros2
}  // namespace tobas

inline rclcpp::Duration operator-(const builtin_interfaces::msg::Time& _lhs, const builtin_interfaces::msg::Time& _rhs)
{
  const auto lhs_ns = RCL_S_TO_NS(_lhs.sec) + _lhs.nanosec;
  const auto rhs_ns = RCL_S_TO_NS(_rhs.sec) + _rhs.nanosec;
  const auto dur_ns = lhs_ns - rhs_ns;
  return rclcpp::Duration::from_nanoseconds(dur_ns);
}

inline bool operator>(const builtin_interfaces::msg::Duration& _lhs, const std::chrono::nanoseconds& _rhs)
{
  return tobas::ros2::nanoseconds(_lhs) > _rhs.count();
}
