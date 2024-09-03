#include "tobas_setup_assistant/setting_tabs/hardware/aso.hpp"

namespace gui
{
namespace setup_assistant
{
const char* AsoWidget::name() const
{
  return "Aso | Tobas";
}

const char* AsoWidget::description() const
{
  return "";  // TODO
}

const char* AsoWidget::hardwarePackage() const
{
  return "tobas_aso_ros";
}

YAML::Node AsoWidget::dump() const
{
  return YAML::Node();
}

void AsoWidget::load(const YAML::Node&)
{
}

bool AsoWidget::isValid()
{
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
