// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas::tim
{
class Stopwatch
{
public:
  explicit Stopwatch(size_t _samples = 1);

  void start();
  uint64_t stop();

  bool isRunning() const;

private:
  const size_t samples_;

  bool running_ = false;
  size_t count_ = 0;
  uint64_t sum_duration_ = 0;
  std::chrono::steady_clock::time_point start_time_;
};
}  // namespace tobas::tim
