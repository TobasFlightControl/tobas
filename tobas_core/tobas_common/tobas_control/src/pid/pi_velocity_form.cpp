#include "tobas_control/pid/pi_velocity_form.hpp"

#include <algorithm>

namespace tobas
{
namespace ctrl
{

PIVelocityForm::PIVelocityForm() : p_(0.0), i_(0.0)
{
}

PIVelocityForm::PIVelocityForm(const double& p, const double& i) : p_(p), i_(i)
{
}

void PIVelocityForm::setGains(const double& p, const double& i)
{
  p_ = p;
  i_ = i;
}

void PIVelocityForm::setMaxMin(const double& max, const double& min)
{
  max_ = max;
  min_ = min;
}

void PIVelocityForm::update(const double& u, const double& dt)
{
  input_ = std::clamp(input_ + ((p_ * (u - last_u_) / dt + i_ * u) * dt), min_, max_);
  last_u_ = u;
}

double PIVelocityForm::getValue()
{
  return input_;
}

}  // namespace ctrl
}  // namespace tobas
