#include "tobas_dsp/utils.hpp"

#include <cassert>
#include <cmath>

using namespace std;

namespace dsp
{
double prewarp(double wc, double dt)
{
  assert(wc > 0.);
  assert(dt > 0.);

  const auto dt_2 = dt / 2.;
  return tan(wc * dt_2) / dt_2;
}
}  // namespace dsp
