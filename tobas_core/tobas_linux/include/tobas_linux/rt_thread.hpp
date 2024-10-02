#pragma once

#include <cstdint>
#include <sys/types.h>
#include <sched.h>

namespace linux
{
int setThreadPriority(pid_t pid, size_t priority, int policy);
int setThisThreadPriority(size_t priority, int policy);

int setThreadCPUAffinity(pid_t pid, uint32_t cpu_bit_mask);
int setThisThreadCPUAffinity(uint32_t cpu_bit_mask);
}  // namespace linux
