// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sched.h>

#include <cstddef>
#include <cstdint>
#include <vector>

#include <tobas_linux/types.hpp>

namespace tobas
{
class MultiComponentManagers
{
public:
  explicit MultiComponentManagers(size_t _num_managers);

  void setPolicy(size_t _idx, linux::sched_t _policy);
  void setPriority(size_t _idx, size_t _priority);
  void setCpuAffinity(size_t _idx, uint32_t _affinity);
  void setNumThreads(size_t _idx, size_t _num_threads);

  void spin();

private:
  struct ManagerConfig
  {
    linux::sched_t policy = SCHED_FIFO;
    size_t priority = 0;
    uint32_t affinity = 0;
    size_t num_threads = 1;
  };

  const size_t num_managers_;
  std::vector<ManagerConfig> configs_;
};
}  // namespace tobas
