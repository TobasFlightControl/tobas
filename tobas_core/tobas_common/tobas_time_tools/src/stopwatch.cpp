// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_time_tools/stopwatch.hpp"

#include <iostream>

namespace ch = std::chrono;

namespace tobas
{
namespace tim
{
Stopwatch::Stopwatch(size_t samples) : samples_(samples)
{
  if (samples == 0) {
    throw std::runtime_error("The number of samples must be positive.");
  }
}

void Stopwatch::start()
{
  start_time_ = ch::steady_clock::now();
  running_ = true;
}

uint64_t Stopwatch::stop()
{
  if (!running_) {
    return 0;
  }

  const auto end_time = ch::steady_clock::now();
  const auto duration = duration_cast<ch::microseconds>(end_time - start_time_).count();
  sum_duration_ += duration;

  if (++count_ == samples_) {
    const auto mean_duration = sum_duration_ / samples_;
    std::cout << "The average duration of " << count_ << " measurements [us]: " << mean_duration << std::endl;
    count_ = 0;
    sum_duration_ = 0;
  }

  return duration;
}

bool Stopwatch::isRunning() const
{
  return running_;
}
}  // namespace tim
}  // namespace tobas
