#include "tobas_std_tools/rate.hpp"

#include <thread>

using namespace std;
using namespace std::chrono;

namespace tobas_std
{
Rate::Rate(const microseconds& period) : period_(period)
{
  if (period.count() <= 0) {
    throw runtime_error("Period must be positive.");
  }

  start();
}

Rate::Rate(const double& freq) : period_(static_cast<uint64_t>(1e+6 / freq))
{
  if (freq <= 0) {
    throw runtime_error("Frequency must be positive.");
  }

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
