#include "tobas_setup_assistant/setting_tabs/hardware/t1.hpp"

#include <QVBoxLayout>

namespace gui
{
namespace sa
{
T1Widget::T1Widget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  rows->addStretch();
}

const char* T1Widget::name() const
{
  return "T1 | Tobas";
}

const char* T1Widget::description() const
{
  return "";  // TODO
}

const char* T1Widget::hardwarePackage() const
{
  return "tobas_t1_ros";
}

YAML::Node T1Widget::dump() const
{
  return YAML::Node(YAML::NodeType::Map);
}

void T1Widget::load(const YAML::Node&)
{
}

bool T1Widget::isValid()
{
  return true;
}
}  // namespace sa
}  // namespace gui
