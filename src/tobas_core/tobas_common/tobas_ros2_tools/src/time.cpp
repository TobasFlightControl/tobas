// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_ros2_tools/time.hpp"

namespace ch = std::chrono;

namespace tobas::ros2
{
namespace
{
constexpr auto kNanosecondsPerSecond = 1'000'000'000;
}  // namespace

void timeChronoToMsg(const ch::steady_clock::duration& _c, builtin_interfaces::msg::Time& _m)
{
  const auto nsec = _c.count();
  assert(nsec >= 0);
  _m.sec = nsec / kNanosecondsPerSecond;
  _m.nanosec = nsec % kNanosecondsPerSecond;
}
}  // namespace tobas::ros2
