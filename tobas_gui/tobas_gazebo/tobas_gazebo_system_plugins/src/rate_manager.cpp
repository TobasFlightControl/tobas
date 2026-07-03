// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/rate_manager.hpp"

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace gazebo
{
RateManager::RateManager(int update_rate) : update_rate_(update_rate), next_time_(0ns)
{
}

bool RateManager::update(const ch::steady_clock::duration& cur_time)
{
  if (update_rate_ <= 0) {
    return true;
  }

  if (cur_time < next_time_) {
    return false;
  }
  else {
    const ch::nanoseconds period(1'000'000'000 / update_rate_);
    if (cur_time - next_time_ < period) {
      next_time_ += period;  // Keep the frequency by updating based on the next scheduled time.
    }
    else {
      next_time_ = cur_time + period;  // Reset based on the current time if two or more periods have elapsed.
    }
    return true;
  }
}
}  // namespace gazebo
}  // namespace tobas
