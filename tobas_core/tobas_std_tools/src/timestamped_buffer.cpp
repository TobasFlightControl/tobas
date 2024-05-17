#include <limits>

#include "../include/tobas_std_tools/timestamped_buffer.hpp"
#include "../include/tobas_std_tools/math.hpp"

using namespace std;

namespace tobas_std
{
double TimestampedBufferDouble::max() const
{
  double res = numeric_limits<double>::lowest();
  for (const auto& [_, x] : map_)
    res = ::max(res, x);
  return res;
}

double TimestampedBufferDouble::min() const
{
  double res = numeric_limits<double>::max();
  for (const auto& [_, x] : map_)
    res = ::min(res, x);
  return res;
}

double TimestampedBufferDouble::range() const
{
  return this->max() - this->min();
}

double TimestampedBufferDouble::mean() const
{
  if (this->size() == 0)
    return 0.;

  double sum = 0.;
  for (const auto& [_, x] : map_)
    sum += x;
  return sum / this->size();
}

double TimestampedBufferDouble::variance() const
{
  if (this->size() == 0)
    return 0.;

  const auto mean = this->mean();
  double sum = 0.;
  for (const auto& [_, x] : map_)
    sum += sqr(x - mean);
  return sum / this->size();
}

double TimestampedBufferDouble::stddev() const
{
  return sqrt(this->variance());
}
}  // namespace tobas_std
