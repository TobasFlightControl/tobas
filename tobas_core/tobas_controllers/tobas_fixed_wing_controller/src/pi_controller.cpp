#include "tobas_fixed_wing_controller/pi_controller.hpp"

#include <algorithm>

namespace tobas
{
namespace fixed_wing
{

PIController::PIController()
: p_(0.0), i_(0.0)
{}

PIController::PIController(const double& p, const double& i)
: p_(p), i_(i)
{}

void PIController::setGains(const double& p, const double& i)
{
  p_ = p;
  i_ = i;
}

void PIController::setMaxMin(const double& max, const double& min)
{
  max_ = max;
  min_ = min;
}

void PIController::update(const double& u, const double& dt)
{
  input_ = std::clamp(input_ + ((p_ * (u - last_u_) / dt + i_ * u) * dt), min_, max_);
  last_u_ = u;
}

double PIController::getValue()
{
  return input_;
}

} // namespace fixed_wing
} // namespace tobas
