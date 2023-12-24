#include <cmath>

#include "../include/dh_std_tools/first_order_filter.hpp"

namespace dh_std
{
double timeConstFromCutoffFreq(const double& cutoff_freq)
{
  assert(cutoff_freq > 0.);
  return 0.5 / M_PI / cutoff_freq;
}
}  // namespace dh_std
