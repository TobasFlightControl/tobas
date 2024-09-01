#include <tobas_qt_tools/message.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/electrodynamics/no_select.hpp"

namespace gui
{
namespace setup_assistant
{
namespace propulsion_system
{
const char* ElectrodynamicsWidget_NoSelect::name() const
{
  return "Select Setting Method";
}

const char* ElectrodynamicsWidget_NoSelect::description() const
{
  return "";
}

void ElectrodynamicsWidget_NoSelect::onInit()
{
}

bool ElectrodynamicsWidget_NoSelect::isValid()
{
  qt::qErrorBox(this, "Please select electrodynamics setting method.");
  return false;
}

void ElectrodynamicsWidget_NoSelect::copyFrom(const ElectrodynamicsWidget_Base*)
{
}

YAML::Node ElectrodynamicsWidget_NoSelect::dump() const
{
  return YAML::Node();
}

void ElectrodynamicsWidget_NoSelect::load(const YAML::Node&)
{
}

std::pair<double, double> ElectrodynamicsWidget_NoSelect::rotSpeedCoefs() const
{
  throw;
}
}  // namespace propulsion_system
}  // namespace setup_assistant
}  // namespace gui
