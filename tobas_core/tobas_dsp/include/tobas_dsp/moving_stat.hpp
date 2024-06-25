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

  inline double mean() const;
  inline double variance() const;

private:
  size_t size_;
  std::deque<double> que_;
  double m_;  // Mean
  double v_;  // Variance
};

inline double MovingStatistics::mean() const
{
  return m_;
}

inline double MovingStatistics::variance() const
{
  return v_;
}
}  // namespace dsp
