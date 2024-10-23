#pragma once

#include <cstdint>
#include <sys/types.h>
#include <sched.h>

namespace linux
{
bool setThreadPriority(pthread_t thread, size_t priority, int policy);
bool setProcessPriority(pid_t pid, size_t priority, int policy);
bool setThisProcessPriority(size_t priority, int policy);

bool setThreadCPUAffinity(pthread_t thread, uint32_t cpu_bit_mask);
bool setProcessCPUAffinity(pid_t pid, uint32_t cpu_bit_mask);
bool setThisProcessCPUAffinity(uint32_t cpu_bit_mask);
}  // namespace linux
