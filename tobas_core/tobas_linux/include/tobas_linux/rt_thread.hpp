#pragma once

#include <cstdint>
#include <sys/types.h>
#include <sched.h>

namespace linux
{
bool setProcessPriority(pid_t pid, size_t priority, int policy);
bool setThisProcessPriority(size_t priority, int policy);

bool setProcessCPUAffinity(pid_t pid, uint32_t cpu_bit_mask);
bool setThisProcessCPUAffinity(uint32_t cpu_bit_mask);
}  // namespace linux
