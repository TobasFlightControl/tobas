#include "../include/tobas_dsp/welford.hpp"

namespace dsp
{
Welford::Welford()
{
  reset();
}

void Welford::reset()
{
  n_ = 0;
  mean_ = 0.;
  var_n_ = 0.;
}

void Welford::addData(const double& x)
{
  ++n_;
  const auto d = x - mean_;
  mean_ += d / n_;
  const auto d2 = x - mean_;
  var_n_ += d * d2;
}
}  // namespace dsp
