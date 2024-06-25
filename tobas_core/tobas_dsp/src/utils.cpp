#include <cmath>
#include <stdexcept>

#include "../include/tobas_dsp/utils.hpp"

using namespace std;

namespace dsp
{
double timeConstFromCutoff(const double& fc)
{
  if (fc <= 0)
    throw runtime_error("Cutoff frequency must be positive.");

  return 1 / (2 * M_PI * fc);
}
}  // namespace dsp
