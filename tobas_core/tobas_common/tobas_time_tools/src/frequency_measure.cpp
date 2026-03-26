#include "tobas_time_tools/frequency_measure.hpp"

#include <iostream>

namespace ch = std::chrono;

namespace tobas
{
namespace tim
{
FrequencyMeasure::FrequencyMeasure(const ch::nanoseconds& period) : period_(period)
{
}

void FrequencyMeasure::count()
{
  // Initialize
  if (count_ == 0) {
    start_time_ = ch::steady_clock::now();
  }

  // Increase count
  ++count_;

  // Output
  const auto end_time = ch::steady_clock::now();
  const auto dur = end_time - start_time_;
  if (dur > period_) {
    const auto freq = count_ / ch::duration<double>(dur).count();
    std::cout << freq << " Hz" << std::endl;

    // Reset
    count_ = 0;
  }
}
}  // namespace tim
}  // namespace tobas
