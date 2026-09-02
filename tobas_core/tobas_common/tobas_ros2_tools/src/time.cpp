// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/time.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace ros2
{
namespace
{
constexpr auto kNanosecondsPerSecond = 1'000'000'000;
}  // namespace

void timeChronoToMsg(const ch::steady_clock::duration& c, builtin_interfaces::msg::Time& m)
{
  const auto nsec = c.count();
  assert(nsec >= 0);
  m.sec = nsec / kNanosecondsPerSecond;
  m.nanosec = nsec % kNanosecondsPerSecond;
}
}  // namespace ros2
}  // namespace tobas
