// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/realtime.hpp"

#include <pthread.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

#include <tobas_std_tools/error.hpp>

namespace tobas
{
namespace linux
{
bool setThreadPriority(pthread_t _thread, size_t _priority, sched_t _policy)
{
  struct sched_param param;
  std::memset(&param, 0, sizeof(param));
  param.sched_priority = _priority;

  if (pthread_setschedparam(_thread, _policy, &param) != 0) {
    std::cerr << "Failed to set scheduling policy: " << st::strError() << '\n';
    return false;
  }

  return true;
}

bool setProcessPriority(pid_t _pid, size_t _priority, sched_t _policy)
{
  struct sched_param param;
  std::memset(&param, 0, sizeof(param));
  param.sched_priority = _priority;

  if (sched_setscheduler(_pid, _policy, &param) != 0) {
    std::cerr << "Failed to set scheduling policy: " << st::strError() << '\n';
    return false;
  }

  return true;
}

bool setThisProcessPriority(size_t _priority, sched_t _policy)
{
  return setProcessPriority(getpid(), _priority, _policy);
}

bool setThreadCPUAffinity(pthread_t _thread, uint32_t _cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0;
  CPU_ZERO(&set);
  while (_cpu_bit_mask > 0) {
    if (_cpu_bit_mask & 1) {
      CPU_SET(cpu_cnt, &set);
    }
    _cpu_bit_mask >>= 1;
    ++cpu_cnt;
  }

  if (pthread_setaffinity_np(_thread, sizeof(set), &set) != 0) {
    std::cerr << "Failed to set CPU affinity: " << st::strError() << '\n';
    return false;
  }

  return true;
}

bool setProcessCPUAffinity(pid_t _pid, uint32_t _cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0;
  CPU_ZERO(&set);
  while (_cpu_bit_mask > 0) {
    if ((_cpu_bit_mask & 1) > 0) {
      CPU_SET(cpu_cnt, &set);
    }
    _cpu_bit_mask >>= 1;
    ++cpu_cnt;
  }

  if (sched_setaffinity(_pid, sizeof(set), &set) != 0) {
    std::cerr << "Failed to set CPU affinity: " << st::strError() << '\n';
    return false;
  }

  return true;
}

bool setThisProcessCpuAffinity(uint32_t _cpu_bit_mask)
{
  return setProcessCPUAffinity(getpid(), _cpu_bit_mask);
}
}  // namespace linux
}  // namespace tobas
