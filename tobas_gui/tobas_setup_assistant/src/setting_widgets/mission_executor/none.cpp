#include "tobas_setup_assistant/setting_tabs/mission_executor/none.hpp"

namespace tobas
{
namespace gui
{
namespace sa
{
namespace mission
{
NoneWidget::NoneWidget()
{
}

QString NoneWidget::executorPackage() const
{
  return "tobas_dummy_pkg";
}

QString NoneWidget::pluginName() const
{
  return "tobas::DummyNode";
}

YAML::Node NoneWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node NoneWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void NoneWidget::load(const YAML::Node&)
{
}

bool NoneWidget::isValid()
{
  return true;
}
}  // namespace mission
}  // namespace sa
}  // namespace gui
}  // namespace tobas
