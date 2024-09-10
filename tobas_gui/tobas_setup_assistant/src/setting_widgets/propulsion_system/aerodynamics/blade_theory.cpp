#include "tobas_setup_assistant/setting_tabs/propulsion_system/aerodynamics/blade_theory.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
AerodynamicsWidget_BladeTheory::AerodynamicsWidget_BladeTheory(PropellerWidget* propeller) : propeller_(propeller)
{
}

const char* AerodynamicsWidget_BladeTheory::name() const
{
  return "Estimate from Propeller Geometry";
}

const char* AerodynamicsWidget_BladeTheory::description() const
{
  return "Estimate aerodynamic constants using Blade Element Theory or Momentum Theory, "
         "based on the geometric shape of the propeller set above. See "
         "<a href='https://en.wikipedia.org/wiki/Blade_element_theory'>Blade Element Theory</a> and "
         "<a href='https://en.wikipedia.org/wiki/Momentum_theory'>Momentum Theory</a> "
         "for more information.";
}

void AerodynamicsWidget_BladeTheory::onInit()
{
}

bool AerodynamicsWidget_BladeTheory::isValid()
{
  return true;
}

void AerodynamicsWidget_BladeTheory::copyFrom(const AerodynamicsWidget_Base*)
{
}

YAML::Node AerodynamicsWidget_BladeTheory::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void AerodynamicsWidget_BladeTheory::load(const YAML::Node&)
{
}

double AerodynamicsWidget_BladeTheory::motorConst() const
{
  return bladeTheory().motorConst();
}

double AerodynamicsWidget_BladeTheory::momentConst() const
{
  return bladeTheory().momentConst();
}

double AerodynamicsWidget_BladeTheory::rotorDragCoef() const
{
  return bladeTheory().rotorDragCoef();
}

BladeTheory AerodynamicsWidget_BladeTheory::bladeTheory() const
{
  return BladeTheory(propeller_->numBlade(), propeller_->radius(), propeller_->bladeChord(), propeller_->pitchAngle());
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
