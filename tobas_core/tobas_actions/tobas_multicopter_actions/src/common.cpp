#include "tobas_multicopter_actions/common.hpp"

#include <algorithm>
#include <cassert>

double approachZeroLinear(double x0, double v, double t)
{
  assert(v >= 0.);
  assert(t >= 0.);

  if (x0 > 0) {
    return std::max(x0 - v * t, 0.);
  }
  else {
    return std::min(x0 + v * t, 0.);
  }
}
