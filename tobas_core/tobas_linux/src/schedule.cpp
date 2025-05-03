#include <iostream>
#include <pthread.h>
#include <sched.h>

#include "../include/tobas_linux/schedule.hpp"
#include "../include/tobas_linux/core.hpp"

using namespace std;

namespace linux
{
bool checkRealtimePriority(const pthread_t& thread, const int& tar_policy, const int& tar_priority)
{
  int cur_policy;
  sched_param cur_param;

  if (pthread_getschedparam(thread, &cur_policy, &cur_param) != 0) {
    cerr << "Failed to get scheduling parameters." << endl;
    return false;
  }

  if (cur_policy != tar_policy) {
    cerr << "Scheduling policy is not reflected." << endl;
    return false;
  }

  if (cur_param.sched_priority != tar_priority) {
    cerr << "Scheduling priority is not reflected." << endl;
    return false;
  }

  return true;
}

bool setRealtimePriority(const int& tar_policy, const int& tar_priority)
{
  if (!isSuperUser()) {
    cerr << "Root privileges are required to set real-time priority." << endl;
    return false;
  }

  if (tar_priority < 0 || 99 < tar_priority) {
    cerr << "Real-time priority must be betwenn 0 and 99." << endl;
    return false;
  }

  const auto this_thread = pthread_self();

  sched_param tar_param;
  tar_param.sched_priority = tar_priority;

  if (pthread_setschedparam(this_thread, tar_policy, &tar_param) != 0) {
    cerr << "Failed to set scheduling parameters." << endl;
    return false;
  }

  if (!checkRealtimePriority(this_thread, tar_policy, tar_priority)) {
    return false;
  }

  return true;
}

bool setRealtimePriorityFIFO(const int& priority)
{
  return setRealtimePriority(SCHED_FIFO, priority);
}

bool setRealtimePriorityRR(const int& priority)
{
  return setRealtimePriority(SCHED_RR, priority);
}
}  // namespace linux
