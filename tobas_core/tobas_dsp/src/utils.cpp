#include <cmath>
#include <stdexcept>

#include "../include/tobas_dsp/utils.hpp"

using namespace std;

namespace dsp
{
double timeConstFromCutoff(const double& cutoff_freq)
{
  if (cutoff_freq <= 0)
    throw runtime_error("Cutoff frequency must be positive.");

  return 0.5 / M_PI / cutoff_freq;
}
}  // namespace dsp
