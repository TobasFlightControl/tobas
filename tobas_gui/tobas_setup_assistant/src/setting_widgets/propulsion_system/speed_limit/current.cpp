#include <tobas_math/core.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/current.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
SpeedLimitWidget_Current::SpeedLimitWidget_Current(
  ElectrodynamicsWidget* electrodynamics,
  AerodynamicsWidget* aerodynamics)
  : electrodynamics_(electrodynamics), aerodynamics_(aerodynamics)
{
}

const char* SpeedLimitWidget_Current::name() const
{
  return "Estimate from Maximum Continuous Current";
}

void SpeedLimitWidget_Current::onInit()
{
  spinbox_->setDecimals(2);
  spinbox_->setMinimum(0.);
  spinbox_->setValue(10.);
  spinbox_->setSuffix(" A");
}

bool SpeedLimitWidget_Current::isValid()
{
  return true;
}

double SpeedLimitWidget_Current::maxRotSpeed() const
{
  const auto [kt, _] = electrodynamics_->rotSpeedCoefs();
  const auto motor_const = aerodynamics_->motorConst();
  const auto moment_const = aerodynamics_->momentConst();

  const auto max_current = spinbox_->value();
  const auto max_torque = kt * max_current;
  const auto max_thrust = max_torque / moment_const;

  return sqrt(max_thrust / motor_const);
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
