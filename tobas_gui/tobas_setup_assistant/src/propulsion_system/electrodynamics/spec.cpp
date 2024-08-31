#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/spec.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
ElectrodynamicsWidget_Spec::ElectrodynamicsWidget_Spec(MotorWidget* motor, AerodynamicsWidget* aerodynamics)
  : motor_(motor), aerodynamics_(aerodynamics)
{
}

const char* ElectrodynamicsWidget_Spec::name() const
{
  return "Estimate from Motor Spec";
}

const char* ElectrodynamicsWidget_Spec::description() const
{
  return "Estimate the motor dynamics from the motor's Kv value and internal registance.";
}

void ElectrodynamicsWidget_Spec::onInit()
{
}

bool ElectrodynamicsWidget_Spec::isValid()
{
  return true;
}

void ElectrodynamicsWidget_Spec::copyFrom(const ElectrodynamicsWidget_Base*)
{
}

YAML::Node ElectrodynamicsWidget_Spec::dump() const
{
  return YAML::Node();
}

void ElectrodynamicsWidget_Spec::load(const YAML::Node&)
{
}

std::pair<double, double> ElectrodynamicsWidget_Spec::rotSpeedCoefs() const
{
  const auto kv_si = motor_->kv();
  const auto resistance = motor_->internalResistance();
  const auto ct = aerodynamics_->motorConst();
  const auto cm = aerodynamics_->momentConst();

  // 発電係数とトルク定数の関係: https://en.wikipedia.org/wiki/Motor_constants
  const auto ke = 1. / kv_si;  // 発電係数 [Vs/rad]
  const auto kt = 1. / kv_si;  // トルク定数 [Nm/A]

  return { ke, resistance * ct * cm / kt };
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
