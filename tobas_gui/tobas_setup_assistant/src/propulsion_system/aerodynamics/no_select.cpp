#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/aerodynamics/no_select.hpp"

namespace gui
{
namespace setup_assistant
{
const char* AerodynamicsWidget_NoSelect::name() const
{
  return "Select Setting Method";
}

const char* AerodynamicsWidget_NoSelect::description() const
{
  return "";
}

void AerodynamicsWidget_NoSelect::onInit()
{
}

bool AerodynamicsWidget_NoSelect::isValid()
{
  qt::qErrorBox(this, "Please select aerodynamics setting method.");
  return false;
}

void AerodynamicsWidget_NoSelect::copyFrom(const AerodynamicsWidget_Base*)
{
}

YAML::Node AerodynamicsWidget_NoSelect::dump()
{
  return YAML::Node();
}

void AerodynamicsWidget_NoSelect::load(const YAML::Node&)
{
}

double AerodynamicsWidget_NoSelect::motorConst() const
{
  throw;
}

double AerodynamicsWidget_NoSelect::momentConst() const
{
  throw;
}

double AerodynamicsWidget_NoSelect::rotorDragCoef() const
{
  throw;
}
}  // namespace setup_assistant
}  // namespace gui
