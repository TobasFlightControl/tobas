#include <iostream>

#include "../include/dh_std_tools/stopwatch.hpp"

using namespace std;
using namespace chrono;

namespace dh_std
{
Stopwatch::Stopwatch(size_t samples) : samples_(samples)
{
  if (samples == 0)
    throw runtime_error("The number of samples must be positive.");
}

void Stopwatch::start()
{
  if (running_)
    throw runtime_error("Stopwatch is still running.");

  start_time_ = system_clock::now();
  running_ = true;
}

uint64_t Stopwatch::stop()
{
  if (!running_)
    throw runtime_error("Stopwatch is not running.");
  running_ = false;

  const auto end_time = system_clock::now();
  const auto duration = duration_cast<microseconds>(end_time - start_time_).count();
  sum_duration += duration;

  if (++count_ == samples_)
  {
    const auto mean_duration = sum_duration / samples_;
    cout << "The average duration of " << count_ << " measurements [us]: " << mean_duration << endl;
    count_ = 0;
    sum_duration = 0;
  }

  return duration;
}

bool Stopwatch::isRunning() const
{
  return running_;
}
}  // namespace dh_std
