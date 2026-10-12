// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_linux/schedule.hpp"

#include <pthread.h>
#include <sched.h>

#include <iostream>

#include "tobas_linux/core.hpp"

namespace tobas
{
namespace linux
{
bool checkRealtimePriority(pthread_t thread, int tar_policy, int tar_priority)
{
  int cur_policy;
  sched_param cur_param;

  if (pthread_getschedparam(thread, &cur_policy, &cur_param) != 0) {
    std::cerr << "Failed to get scheduling parameters." << std::endl;
    return false;
  }

  if (cur_policy != tar_policy) {
    std::cerr << "Scheduling policy is not reflected." << std::endl;
    return false;
  }

  if (cur_param.sched_priority != tar_priority) {
    std::cerr << "Scheduling priority is not reflected." << std::endl;
    return false;
  }

  return true;
}

bool setRealtimePriority(int tar_policy, int tar_priority)
{
  if (!isSuperUser()) {
    std::cerr << "Root privileges are required to set real-time priority." << std::endl;
    return false;
  }

  if (tar_priority < 0 || 99 < tar_priority) {
    std::cerr << "Real-time priority must be between 0 and 99." << std::endl;
    return false;
  }

  const auto this_thread = pthread_self();

  sched_param tar_param;
  tar_param.sched_priority = tar_priority;

  if (pthread_setschedparam(this_thread, tar_policy, &tar_param) != 0) {
    std::cerr << "Failed to set scheduling parameters." << std::endl;
    return false;
  }

  if (!checkRealtimePriority(this_thread, tar_policy, tar_priority)) {
    return false;
  }

  return true;
}

bool setRealtimePriorityFIFO(int priority)
{
  return setRealtimePriority(SCHED_FIFO, priority);
}

bool setRealtimePriorityRR(int priority)
{
  return setRealtimePriority(SCHED_RR, priority);
}
}  // namespace linux
}  // namespace tobas
