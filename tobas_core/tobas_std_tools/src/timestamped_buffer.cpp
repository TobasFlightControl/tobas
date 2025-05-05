#include "../include/tobas_std_tools/timestamped_buffer.hpp"

#include <cmath>
#include <limits>

#include <tobas_algorithm/kahan.hpp>
#include <tobas_math/core.hpp>

using namespace std;

namespace tobas_std
{
double TimestampedBufferDouble::max() const
{
  double res = -INFINITY;
  for (const auto& [_, x] : map_) {
    res = ::max(res, x);
  }
  return res;
}

double TimestampedBufferDouble::min() const
{
  double res = INFINITY;
  for (const auto& [_, x] : map_) {
    res = ::min(res, x);
  }
  return res;
}

double TimestampedBufferDouble::range() const
{
  return this->max() - this->min();
}

double TimestampedBufferDouble::mean() const
{
  if (this->size() == 0) {
    return 0.;
  }

  algo::Kahan<double> sum;
  for (const auto& [_, x] : map_) {
    sum.add(x);
  }

  return sum.get() / this->size();
}

double TimestampedBufferDouble::variance() const
{
  if (this->size() == 0) {
    return 0.;
  }

  const auto mean = this->mean();

  algo::Kahan<double> sum;
  for (const auto& [_, x] : map_) {
    sum.add(math::sqr(x - mean));
  }

  return sum.get() / this->size();
}

double TimestampedBufferDouble::stddev() const
{
  return sqrt(this->variance());
}
}  // namespace tobas_std
