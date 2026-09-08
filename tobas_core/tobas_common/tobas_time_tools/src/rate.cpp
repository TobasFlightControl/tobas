// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_time_tools/rate.hpp"

#include <thread>

using namespace std::chrono_literals;
namespace ch = std::chrono;

namespace tobas
{
namespace tim
{
Rate::Rate()
{
}

Rate::Rate(const ch::microseconds& period)
{
  setInterval(period);
  start();
}

Rate::Rate(const double& freq) : period_(static_cast<uint64_t>(1e+6 / freq))
{
  if (freq <= 0.0) {
    throw std::runtime_error("Frequency must be positive.");
  }

  start();
}

void Rate::start()
{
  last_time_ = ch::steady_clock::now();
}

void Rate::sleep()
{
  const auto next_time = last_time_ + period_;
  std::this_thread::sleep_until(next_time);
  last_time_ = next_time;
}

void Rate::setInterval(const ch::microseconds& _period)
{
  period_ = std::max(_period, 0us);
}
}  // namespace tim
}  // namespace tobas
