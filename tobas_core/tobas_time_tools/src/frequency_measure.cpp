#include "tobas_time_tools/frequency_measure.hpp"

#include <iostream>

using namespace std;
using namespace chrono;

namespace tim
{
FrequencyMeasure::FrequencyMeasure(const std::chrono::nanoseconds& period) : period_(period)
{
}

void FrequencyMeasure::count()
{
  if (count_ == 0) {
    start_time_ = steady_clock::now();
  }

  ++count_;

  const auto end_time = steady_clock::now();
  const auto dur = end_time - start_time_;

  if (dur > period_) {
    const auto freq = count_ / duration<double>(dur).count();
    cout << freq << " Hz" << endl;

    // Reset
    count_ = 0;
  }
}
}  // namespace tim
