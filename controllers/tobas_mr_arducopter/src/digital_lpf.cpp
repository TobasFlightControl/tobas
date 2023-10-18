#include <cmath>
#include <cassert>
#include <algorithm>

#include "../include/tobas_mr_arducopter/digital_lpf.hpp"
#include "../include/tobas_mr_arducopter/math.hpp"

using namespace std;

namespace tobas_mr_arducopter
{
DigitalLPF::DigitalLPF()
{
}

double DigitalLPF::apply(const double& sample, double cutoff_freq, double dt)
{
  assert(cutoff_freq >= 0 && dt >= 0);

  if (cutoff_freq == 0)
  {
    output_ = sample;
    return output_;
  }
  if (dt == 0)
  {
    return output_;
  }

  const double rc = 1 / (M_2PI * cutoff_freq);
  alpha_ = clamp(dt / (dt + rc), 0., 1.);
  output_ += (sample - output_) * alpha_;
  if (!initialized_)
  {
    initialized_ = true;
    output_ = sample;
  }
  return output_;
}

double DigitalLPF::apply(const double& sample)
{
  output_ += (sample - output_) * alpha_;
  if (!initialized_)
  {
    initialized_ = true;
    output_ = sample;
  }
  return output_;
}

void DigitalLPF::computeAlpha(double sample_freq, double cutoff_freq)
{
  if (sample_freq <= 0)
  {
    alpha_ = 1;
  }
  else
  {
    alpha_ = calcLowPassAlphaDt(1 / sample_freq, cutoff_freq);
  }
}

void DigitalLPF::reset(double value)
{
  output_ = value;
  initialized_ = true;
}
}  // namespace tobas_mr_arducopter
