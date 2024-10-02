#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "../include/tobas_linux/rt_thread.hpp"

namespace linux
{
int setThreadPriority(pid_t pid, size_t priority, int policy)
{
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = priority;
  return sched_setscheduler(pid, policy, &param);
}

int setThisThreadPriority(size_t priority, int policy)
{
  return setThreadPriority(getpid(), priority, policy);
}

int setThreadCPUAffinity(pid_t pid, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0)
  {
    if ((cpu_bit_mask & 1) > 0)
      CPU_SET(cpu_cnt, &set);
    cpu_bit_mask = (cpu_bit_mask >> 1);
    cpu_cnt++;
  }
  return sched_setaffinity(pid, sizeof(set), &set);
}

int setThisThreadCPUAffinity(uint32_t cpu_bit_mask)
{
  return setThreadCPUAffinity(getpid(), cpu_bit_mask);
}
}  // namespace linux
