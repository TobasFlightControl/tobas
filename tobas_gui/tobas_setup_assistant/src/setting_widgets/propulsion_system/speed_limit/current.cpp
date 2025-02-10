#include <tobas_math/core.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/current.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
SpeedLimitWidget_Current::SpeedLimitWidget_Current(MotorWidget* motor, AerodynamicsWidget* aerodynamics)
  : motor_(motor), aerodynamics_(aerodynamics)
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
  const auto kt = 1. / motor_->kv();
  const auto motor_const = aerodynamics_->motorConst();
  const auto moment_const = aerodynamics_->momentConst();

  const auto max_current = spinbox_->value();
  const auto max_torque = kt * max_current;
  const auto max_thrust = max_torque / moment_const;

  return sqrt(max_thrust / motor_const);
}
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
