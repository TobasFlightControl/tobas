#include "../include/tobas_std_tools/timestamped_buffer.hpp"
#include "../include/tobas_std_tools/math.hpp"

namespace tobas_std
{
double TimestampedBufferDouble::mean() const
{
  if (buffer_.size() == 0)
    return 0.;

  double sum = 0.;
  for (const auto& [_, x] : buffer_)
    sum += x;
  return sum / buffer_.size();
}

double TimestampedBufferDouble::variance() const
{
  if (buffer_.size() == 0)
    return 0.;

  const auto mean = this->mean();
  double sum = 0.;
  for (const auto& [_, x] : buffer_)
    sum += sqr(x - mean);
  return sum / buffer_.size();
}

double TimestampedBufferDouble::stddev() const
{
  return sqrt(this->variance());
}
}  // namespace tobas_std
