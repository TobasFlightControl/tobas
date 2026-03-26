#pragma once

#include <sched.h>
#include <sys/types.h>

#include <cstdint>

#include "./types.hpp"

namespace tobas
{
namespace linux
{
bool setThreadPriority(pthread_t thread, size_t priority, sched_t policy);
bool setProcessPriority(pid_t pid, size_t priority, sched_t policy);
bool setThisProcessPriority(size_t priority, sched_t policy);

bool setThreadCPUAffinity(pthread_t thread, uint32_t cpu_bit_mask);
bool setProcessCPUAffinity(pid_t pid, uint32_t cpu_bit_mask);
bool setThisProcessCpuAffinity(uint32_t cpu_bit_mask);
}  // namespace linux
}  // namespace tobas
