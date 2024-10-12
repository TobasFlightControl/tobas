#include <QVBoxLayout>

#include "tobas_setup_assistant/setting_tabs/observer/custom.hpp"

namespace gui
{
namespace setup_assistant
{
CustomObserverWidget::CustomObserverWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

const char* CustomObserverWidget::name() const
{
  return "Use Custom Observer";
}

const char* CustomObserverWidget::description() const
{
  return "";  // TODO: APIの案内など
}

const char* CustomObserverWidget::observerPackage() const
{
  return "tobas_dummy_pkg";
}

YAML::Node CustomObserverWidget::staticParams() const
{
  return YAML::Node(YAML::NodeType::Map);
}

YAML::Node CustomObserverWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void CustomObserverWidget::load(const YAML::Node&)
{
}

bool CustomObserverWidget::isValid()
{
  return true;
}
}  // namespace setup_assistant
}  // namespace gui
