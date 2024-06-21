#pragma once

#include <cstddef>
#include <vector>
#include <deque>

namespace dsp
{
class MovingStatistics
{
public:
  explicit MovingStatistics();

  void initialize(const std::vector<double>& data);
  void add(double x);

  inline double mean();
  inline double variance();

private:
  size_t size_;
  std::deque<double> que_;
  double m_;  // Mean
  double v_;  // Variance
};

inline double MovingStatistics::mean()
{
  return m_;
}

inline double MovingStatistics::variance()
{
  return v_;
}
}  // namespace dsp
