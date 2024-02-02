#include "../include/tobas_std_tools/statistics.hpp"

namespace tobas_std
{
OnlineStatistics::OnlineStatistics()
{
  reset();
}

void OnlineStatistics::reset()
{
  n_ = 0;
  mean_ = 0.;
  m2_ = 0.;
}

void OnlineStatistics::addData(const double& x)
{
  ++n_;
  const auto delta = x - mean_;
  mean_ += delta / n_;
  const auto delta2 = x - mean_;
  m2_ += delta * delta2;
}
}  // namespace tobas_std
