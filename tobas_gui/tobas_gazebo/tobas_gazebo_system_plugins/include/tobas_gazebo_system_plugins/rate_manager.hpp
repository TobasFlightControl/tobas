// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>
#include <memory>

namespace tobas
{
namespace gazebo
{
class RateManager
{
public:
  explicit RateManager(int update_rate);

  /* True if the cycle can be executed. */
  bool update(const std::chrono::steady_clock::duration& cur_time);

private:
  const int update_rate_;
  std::chrono::steady_clock::duration next_time_;
};
}  // namespace gazebo
}  // namespace tobas
