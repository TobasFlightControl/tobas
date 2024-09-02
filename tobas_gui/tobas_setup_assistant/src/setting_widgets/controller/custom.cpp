#include "tobas_setup_assistant/setting_tabs/controller/custom.hpp"

namespace gui
{
namespace setup_assistant
{
const char* CustomControllerWidget::name() const
{
  return "Use Custom Controller";
}

const char* CustomControllerWidget::description() const
{
  return "";  // TODO: APIの案内など
}

const char* CustomControllerWidget::controllerPackage() const
{
  return "tobas_dummy_pkg";
}

const char* CustomControllerWidget::actionsPackage() const
{
  return "tobas_dummy_pkg";
}

tobas::rc_command_t CustomControllerWidget::stabilizeModeCommand() const
{
  return tobas::rc_command_t::PROGRAM;
}

tobas::rc_command_t CustomControllerWidget::acrobatModeCommand() const
{
  return tobas::rc_command_t::PROGRAM;
}

YAML::Node CustomControllerWidget::staticParams() const
{
  return YAML::Node();
}

YAML::Node CustomControllerWidget::dump() const
{
  return YAML::Node();
}

void CustomControllerWidget::load(const YAML::Node&)
{
}

bool CustomControllerWidget::isApplicable()
{
  return true;
}

bool CustomControllerWidget::isValid()
{
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
