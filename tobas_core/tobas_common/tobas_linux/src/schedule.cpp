// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/schedule.hpp"

#include <pthread.h>
#include <sched.h>

#include <iostream>

#include "tobas_linux/core.hpp"

namespace tobas::linux
{
bool checkRealtimePriority(pthread_t _thread, int _tar_policy, int _tar_priority)
{
  int cur_policy;
  sched_param cur_param;

  if (pthread_getschedparam(_thread, &cur_policy, &cur_param) != 0) {
    std::cerr << "Failed to get scheduling parameters.\n";
    return false;
  }

  if (cur_policy != _tar_policy) {
    std::cerr << "Scheduling policy is not reflected.\n";
    return false;
  }

  if (cur_param.sched_priority != _tar_priority) {
    std::cerr << "Scheduling priority is not reflected.\n";
    return false;
  }

  return true;
}

bool setRealtimePriority(int _tar_policy, int _tar_priority)
{
  if (!isSuperUser()) {
    std::cerr << "Root privileges are required to set real-time priority.\n";
    return false;
  }

  if (_tar_priority < 0 || 99 < _tar_priority) {
    std::cerr << "Real-time priority must be between 0 and 99.\n";
    return false;
  }

  const auto this_thread = pthread_self();

  sched_param tar_param;
  tar_param.sched_priority = _tar_priority;

  if (pthread_setschedparam(this_thread, _tar_policy, &tar_param) != 0) {
    std::cerr << "Failed to set scheduling parameters.\n";
    return false;
  }

  if (!checkRealtimePriority(this_thread, _tar_policy, _tar_priority)) {
    return false;
  }

  return true;
}

bool setRealtimePriorityFIFO(int _priority)
{
  return setRealtimePriority(SCHED_FIFO, _priority);
}

bool setRealtimePriorityRR(int _priority)
{
  return setRealtimePriority(SCHED_RR, _priority);
}
}  // namespace tobas::linux
