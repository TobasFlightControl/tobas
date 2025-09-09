#pragma once

#include <chrono>

namespace tim
{
class Rate
{
public:
  explicit Rate(const std::chrono::microseconds& period);
  explicit Rate(const double& freq);

  void start();
  void sleep();

private:
  const std::chrono::microseconds period_;
  std::chrono::steady_clock::time_point last_time_;
};
}  // namespace tim
