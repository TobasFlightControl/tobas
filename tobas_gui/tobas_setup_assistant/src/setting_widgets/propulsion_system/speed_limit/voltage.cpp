#include <tobas_math/core.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/voltage.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
SpeedLimitWidget_Voltage::SpeedLimitWidget_Voltage(ElectrodynamicsWidget* electrodynamics)
  : electrodynamics_(electrodynamics)
{
}

const char* SpeedLimitWidget_Voltage::name() const
{
  return "Estimate from Maximum Input Voltage";
}

void SpeedLimitWidget_Voltage::onInit()
{
  spinbox_->setDecimals(2);
  spinbox_->setMinimum(0.);
  spinbox_->setValue(16.8);
  spinbox_->setSuffix(" V");
}

bool SpeedLimitWidget_Voltage::isValid()
{
  return true;
}

double SpeedLimitWidget_Voltage::maxRotSpeed() const
{
  const auto V = spinbox_->value();
  const auto [a, b] = electrodynamics_->rotSpeedCoefs();
  return (sqrt(math::sqr(a) + 4 * b * V) - a) / (2 * b);
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
