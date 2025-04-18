#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_yaml_tools/convert/qstring.hpp>

#include "tobas_setup_assistant/setting_tabs/propulsion_system/ice/engine/limit.hpp"

namespace gui
{
namespace sa
{
namespace propulsion
{
namespace ice
{
EngineLimitWidget::EngineLimitWidget()
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  max_speed_ = new ParamGetterWidget_SpinBox("Maximum Ratation Speed", "");  // TODO
  max_speed_->setMinimum(1);
  max_speed_->setValue(8700);
  max_speed_->setSuffix(" rpm");
  rows->addWidget(max_speed_);

  rows->addStretch();
}

bool EngineLimitWidget::isValid()
{
  return true;
}

YAML::Node EngineLimitWidget::dump() const
{
  YAML::Node node(YAML::NodeType::Map);

  node[max_speed_->name()] = max_speed_->getValue();

  return node;
}

void EngineLimitWidget::load(const YAML::Node& node)
{
  max_speed_->setValue(node[max_speed_->name()].as<int>());
}

double EngineLimitWidget::maxSpeed() const
{
  return tobas_std::rpm2rps(max_speed_->getValue());
}
}  // namespace ice
}  // namespace propulsion
}  // namespace sa
}  // namespace gui
