// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rclcpp/executors/multi_threaded_executor.hpp>

#include <tobas_linux/types.hpp>

namespace tobas
{
/* MultiThreadedExecutorのスレッドプールにリアルタイム優先度を付与． */
class MultiThreadedExecutorRT : public rclcpp::executors::MultiThreadedExecutor
{
public:
  using SharedPtr = std::shared_ptr<MultiThreadedExecutorRT>;

  explicit MultiThreadedExecutorRT(linux::sched_t policy, size_t priority, uint32_t cpu_affinity, size_t num_threads);

  void spin() override;

  linux::sched_t policy() const;
  size_t priority() const;

private:
  const int policy_;
  const size_t priority_;
  const uint32_t cpu_affinity_;
};
}  // namespace tobas
