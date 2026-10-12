// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <sched.h>
#include <sys/types.h>

#include <cstdint>

#include "./types.hpp"

namespace tobas
{
namespace linux
{
bool setThreadPriority(pthread_t _thread, size_t _priority, sched_t _policy);
bool setProcessPriority(pid_t _pid, size_t _priority, sched_t _policy);
bool setThisProcessPriority(size_t _priority, sched_t _policy);

bool setThreadCPUAffinity(pthread_t _thread, uint32_t _cpu_bit_mask);
bool setProcessCPUAffinity(pid_t _pid, uint32_t _cpu_bit_mask);
bool setThisProcessCpuAffinity(uint32_t _cpu_bit_mask);
}  // namespace linux
}  // namespace tobas
