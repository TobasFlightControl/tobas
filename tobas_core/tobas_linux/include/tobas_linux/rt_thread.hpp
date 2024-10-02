#pragma once

#include <cstdint>
#include <sys/types.h>
#include <sched.h>

namespace linux
{
bool setThreadPriority(pid_t pid, size_t priority, int policy);
bool setThisThreadPriority(size_t priority, int policy);

bool setThreadCPUAffinity(pid_t pid, uint32_t cpu_bit_mask);
bool setThisThreadCPUAffinity(uint32_t cpu_bit_mask);
}  // namespace linux
