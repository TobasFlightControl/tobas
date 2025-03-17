#include <QVBoxLayout>

#include "tobas_setup_assistant/setting_tabs/hardware/t1.hpp"

namespace gui
{
namespace sa
{
AsoWidget::AsoWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

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
  return "tobas_t1_ros";
}

YAML::Node AsoWidget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void AsoWidget::load(const YAML::Node&)
{
}

bool AsoWidget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
