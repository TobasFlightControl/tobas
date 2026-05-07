// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

namespace tobas
{
namespace tim
{
class Rate
{
public:
  explicit Rate(const std::chrono::microseconds& period);
  explicit Rate(const double& freq);

  void start();
  void sleep();

private:
  const std::chrono::microseconds period_;
  std::chrono::steady_clock::time_point last_time_;
};
}  // namespace tim
}  // namespace tobas
