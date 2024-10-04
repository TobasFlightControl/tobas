#include <iostream>
#include <string.h>
#include <unistd.h>
#include <pthread.h>

#include "../include/tobas_linux/realtime.hpp"
#include "../include/tobas_linux/errer.hpp"

using namespace std;

namespace linux
{
bool setThreadPriority(pthread_t thread, size_t priority, int policy)
{
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = priority;

  if (pthread_setschedparam(thread, policy, &param) != 0)
  {
    cerr << "Failed to set scheduling policy: " << strError() << endl;
    return false;
  }

  return true;
}

bool setProcessPriority(pid_t pid, size_t priority, int policy)
{
  struct sched_param param;
  memset(&param, 0, sizeof(param));
  param.sched_priority = priority;

  if (sched_setscheduler(pid, policy, &param) != 0)
  {
    cerr << "Failed to set scheduling policy: " << strError() << endl;
    return false;
  }

  return true;
}

bool setThisProcessPriority(size_t priority, int policy)
{
  return setProcessPriority(getpid(), priority, policy);
}

bool setThreadCPUAffinity(pthread_t thread, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0)
  {
    if ((cpu_bit_mask & 1) > 0)
      CPU_SET(cpu_cnt, &set);
    cpu_bit_mask >>= 1;
    ++cpu_cnt;
  }

  if (pthread_setaffinity_np(thread, sizeof(set), &set) != 0)
  {
    cerr << "Failed to set CPU affinity: " << strError() << endl;
    return false;
  }

  return true;
}

bool setProcessCPUAffinity(pid_t pid, uint32_t cpu_bit_mask)
{
  cpu_set_t set;
  uint32_t cpu_cnt = 0;
  CPU_ZERO(&set);
  while (cpu_bit_mask > 0)
  {
    if ((cpu_bit_mask & 1) > 0)
      CPU_SET(cpu_cnt, &set);
    cpu_bit_mask >>= 1;
    ++cpu_cnt;
  }

  if (sched_setaffinity(pid, sizeof(set), &set) != 0)
  {
    cerr << "Failed to set CPU affinity: " << strError() << endl;
    return false;
  }

  return true;
}

bool setThisProcessCPUAffinity(uint32_t cpu_bit_mask)
{
  return setProcessCPUAffinity(getpid(), cpu_bit_mask);
}
}  // namespace linux
