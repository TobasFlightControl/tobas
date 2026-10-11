#include "tobas_fixed_wing_controller/incomplete_differentiation.hpp"

#include <cmath>

namespace tobas
{
namespace fixed_wing
{
IncompleteDifferentiation::IncompleteDifferentiation()
{
}

IncompleteDifferentiation::IncompleteDifferentiation(const double& cutoff_frequency)
{
  setCutoffFrequency(cutoff_frequency);
}

void IncompleteDifferentiation::setCutoffFrequency(const double& freq)
{
  tau_ = 1 / (2.0 * M_PI * freq);
}

void IncompleteDifferentiation::update(const double& u, const double& dt)
{
  theta_ = (tau_ / dt * theta_ + (u - u_last_) / dt) / (1.0 + tau_ / dt);
  u_last_ = u;
}

double IncompleteDifferentiation::getValue()
{
  return theta_;
}

}  // namespace fixed_wing
}  // namespace tobas
