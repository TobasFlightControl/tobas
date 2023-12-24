#pragma once

#include <chrono>

namespace dh_std
{
class Stopwatch
{
public:
  explicit Stopwatch(size_t samples = 1);

  void start();
  uint64_t stop();

  bool isRunning() const;

private:
  const size_t samples_;

  bool running_ = false;
  size_t count_ = 0;
  uint64_t sum_duration = 0;
  std::chrono::system_clock::time_point start_time_;  // [us]
};
}  // namespace dh_std
