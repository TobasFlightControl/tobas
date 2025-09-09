#pragma once

#include <chrono>

namespace tim
{
class FrequencyMeasure
{
public:
  explicit FrequencyMeasure(const std::chrono::nanoseconds& period = std::chrono::seconds(1));

  void count();

private:
  const std::chrono::nanoseconds period_;

  size_t count_ = 0;
  std::chrono::steady_clock::time_point start_time_;
};
}  // namespace tim
