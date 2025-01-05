#include <tobas_math/core.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/speed_limit/voltage.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion
{
SpeedLimitWidget_Voltage::SpeedLimitWidget_Voltage(MotorWidget* motor, AerodynamicsWidget* aerodynamics)
  : motor_(motor), aerodynamics_(aerodynamics)
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
  const auto kv = motor_->kv();
  const auto R = motor_->internalResistance();
  const auto ct = aerodynamics_->motorConst();
  const auto cm = aerodynamics_->momentConst();

  const auto b = kv * R * ct * cm;
  const auto c = 1. / kv;

  const auto V = spinbox_->value();
  return (sqrt(math::sqr(c) + 4 * b * V) - c) / (2 * b);
}
}  // namespace propulsion
}  // namespace setup_assistant
}  // namespace gui
