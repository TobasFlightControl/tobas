#include <thread>

#include "../include/tobas_std_tools/rate.hpp"

using namespace std;
using namespace std::chrono;

namespace tobas_std
{
Rate::Rate(const microseconds& period) : period_(period)
{
  start();
}

void Rate::start()
{
  last_time_ = steady_clock::now();
}

void Rate::sleep()
{
  const auto cur_time = steady_clock::now();
  const auto elapsed_time = cur_time - last_time_;
  const auto wait_time = max(period_ - elapsed_time, nanoseconds(0));
  this_thread::sleep_for(wait_time);
  last_time_ = cur_time + wait_time;
}
}  // namespace tobas_std
