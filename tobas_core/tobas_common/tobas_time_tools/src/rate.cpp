#include "tobas_time_tools/rate.hpp"

#include <thread>

namespace ch = std::chrono;

namespace tim
{
Rate::Rate(const ch::microseconds& period) : period_(period)
{
  if (period.count() <= 0) {
    throw std::runtime_error("Period must be positive.");
  }

  start();
}

Rate::Rate(const double& freq) : period_(static_cast<uint64_t>(1e+6 / freq))
{
  if (freq <= 0) {
    throw std::runtime_error("Frequency must be positive.");
  }

  start();
}

void Rate::start()
{
  last_time_ = ch::steady_clock::now();
}

void Rate::sleep()
{
  const auto cur_time = ch::steady_clock::now();
  const auto elapsed_time = cur_time - last_time_;
  const auto wait_time = std::max(period_ - elapsed_time, ch::nanoseconds(0));
  std::this_thread::sleep_for(wait_time);
  last_time_ = cur_time + wait_time;
}
}  // namespace tim
